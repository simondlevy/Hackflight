/*
   position.cpp : Position-estimation class implementation

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#ifdef __arm__
extern "C" {
#endif

#include "mw.hpp"

static const float CONFIG_BARO_CF_ALT = 0.965;
static const float CONFIG_BARO_CF_VEL = 0.985;

static const uint8_t CONFIG_ALT_P = 50;
static const uint8_t CONFIG_VEL_P = 120;
static const uint8_t CONFIG_VEL_I = 45;
static const uint8_t CONFIG_VEL_D = 1;

static bool sonarInRange(void)
{
    return false; // XXX
}


// complementary filter
static float cfilter(float a, float b, float c) 
{
    return a * c + b * (1 - c);
}

static int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}


void Position::init(Board * _board, IMU * _imu, Baro * _baro)
{
    this->board = _board;
    this->imu = _imu;
    this->baro = _baro;
}

void Position::getAltitude(
        int32_t & estAlt, 
        int32_t & altPID,
        int32_t & errorVelocityI, 
        int32_t setVelocity, 
        bool velocityControl, 
        int32_t altHold,
        bool armed)
{
    static uint32_t previousT;
    static float    accZ_old;
    static float    accelVel;
    static int32_t  fusedBarosonarAlt;
    static int32_t  lastFusedBarosonarAlt;
    static int32_t  baroAlt;
    static int32_t  baroAltBaseline;
    static float    accelAlt;
    static bool     wasArmed;
    static int32_t  baroAlt_offset;
    static float    sonarTransition;

    uint32_t currentT = this->board->getMicros();
    int16_t tiltAngle = max(abs(this->imu->angle[ROLL]), abs(this->imu->angle[PITCH]));
    uint32_t dTime = currentT - previousT;

    if (dTime < CONFIG_ALTITUDE_UPDATE_MSEC*1000) 
        return;

    previousT = currentT;

    int32_t baroAltRaw = this->baro->getAltitude();

    // Grab baro baseline on arming
    if (armed) {
        if (!wasArmed) {
            baroAltBaseline = baroAltRaw;
            accelVel = 0;
            accelAlt = 0;
        }
        baroAlt = baroAltRaw - baroAltBaseline;
    }
    else {
        baroAlt = 0;
    }
    wasArmed = armed;

    // Calculate sonar altitude only if the sonar is facing downwards(<25deg)
    int32_t sonarAlt = -1; // XXX (tiltAngle > 250) ? -1 : sonarAlt * (900.0f - tiltAngle) / 900.0f;

    // Fuse sonarAlt and baroAlt
    if (sonarInRange()) {
        baroAlt_offset = baroAlt - sonarAlt;
        fusedBarosonarAlt = sonarAlt;
    } else {
        baroAlt = baroAlt - baroAlt_offset;
        if (sonarAlt > 0) {
            sonarTransition = (300 - sonarAlt) / 100.0f;
            fusedBarosonarAlt = cfilter(sonarAlt, baroAlt, sonarTransition); 
        }
    }

    printf("%d\n", baroAlt);

    // delta acc reading time in seconds
    float dt = this->imu->accelTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    float accZ_tmp = (float)this->imu->accelSum[2] / (float)this->imu->accelSumCount;
    float vel_acc = accZ_tmp * this->imu->accelVelScale * (float)this->imu->accelTimeSum;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    accelAlt += (vel_acc * 0.5f) * dt + accelVel * dt;                                         
    accelVel += vel_acc;

    // complementary filter for altitude estimation (baro & acc)
    accelAlt = cfilter(accelAlt, fusedBarosonarAlt, CONFIG_BARO_CF_ALT);

    estAlt = sonarInRange() ? fusedBarosonarAlt : accelAlt;

    // reset acceleromter sum
    this->imu->accelSum[0] = 0;
    this->imu->accelSum[1] = 0;
    this->imu->accelSum[2] = 0;
    this->imu->accelSumCount = 0;
    this->imu->accelTimeSum = 0;

    int32_t fusedBaroSonarVel = (fusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime;
    lastFusedBarosonarAlt = fusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = applyDeadband(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    accelVel = cfilter(accelVel, fusedBaroSonarVel, CONFIG_BARO_CF_VEL);
    int32_t vel_tmp = lrintf(accelVel);

    if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)

        int32_t setVel = setVelocity;

        // Altitude P-Controller
        if (!velocityControl) {
            int32_t error = constrain(altHold - estAlt, -500, 500);
            error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - vel_tmp;
        altPID = constrain((CONFIG_VEL_P * error / 32), -300, +300);

        // I
        errorVelocityI += (CONFIG_VEL_I * error);
        errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
        altPID += errorVelocityI / 8196;     // I in the range of +/-200

        // D
        altPID -= constrain(CONFIG_VEL_D * (accZ_tmp + accZ_old) / 512, -150, 150);

    } else {
        altPID = 0;
    }

    accZ_old = accZ_tmp;
}

#ifdef __arm__
} // extern "C"
#endif
