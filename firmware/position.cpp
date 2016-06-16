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

#ifdef __arm__
extern "C" {
#endif

#include "mw.hpp"

#include <math.h>

static const float   CONFIG_BARO_CF_VEL = 0.985;

static const uint8_t CONFIG_ALT_P = 50;
static const uint8_t CONFIG_VEL_P = 120;
static const uint8_t CONFIG_VEL_I = 45;
static const uint8_t CONFIG_VEL_D = 1;



// complementary filter
static float cfilter(float a, float b, float c) 
{
    return a * c + b * (1 - c);
}

static bool sonarInRange(void)
{
    return false; // XXX
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

void Position::init(Baro * _baro, IMU * _imu, bool _velocityControl)
{
    this->baro = _baro;
    this->imu  = _imu;
    this->velocityControl = _velocityControl;

    this->sonarAlt = 0;
    this->previousTime = 0;
    this->accZ_old = 0;
    this->accelVel = 0;
    this->FusedBarosonarAlt = 0;
    this->FusedBarosonarAlt = 0;
    this->baroAlt = 0;
    this->baroAltBaseline = 0;
    this->accelAlt = 0;
    this->wasArmed = 0;
    this->baroAlt_offset = 0;
    this->sonarTransition = 0;
    this->altPID = 0;
    this->errorVelocityI = 0;
}

int32_t Position::getAltitude(bool armed, int32_t altHold, uint32_t currentTime)
{
    this->previousTime = currentTime;

    // Grab baro baseline on arming
    if (armed) {
        int32_t baroAltRaw = this->baro->getAltitude();
        if (!this->wasArmed) {
            this->baroAltBaseline = baroAltRaw;
            this->accelVel = 0;
            this->accelAlt = 0;
        }
        printf("%d %d\n", baroAltRaw, baroAltBaseline);
        this->baroAlt = baroAltRaw - this->baroAltBaseline;
    }
    else {
        this->baroAlt = 0;
    }
    this->wasArmed = armed;

    // Calculate sonar altitude only if the sonar is facing downwards(<25deg)
    int16_t tiltAngle = max(abs(this->imu->angle[ROLL]), abs(this->imu->angle[PITCH]));

    // Fuse sonarAlt and this->baroAlt
    this->sonarAlt = (tiltAngle > 250) ? -1 : sonarAlt * (900.0f - tiltAngle) / 900.0f;
    if (sonarInRange()) {
        this->baroAlt_offset = this->baroAlt - sonarAlt;
        this->FusedBarosonarAlt = sonarAlt;
    } else {
        this->baroAlt = this->baroAlt - this->baroAlt_offset;
        if (sonarAlt > 0) {
            this->sonarTransition = (300 - sonarAlt) / 100.0f;
            this->FusedBarosonarAlt = cfilter(sonarAlt, this->baroAlt, this->sonarTransition); 
        }
    }

    // delta acc reading time in seconds
    float dt = this->imu->accelTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    float accZ_tmp = (float)this->imu->accelSum[2] / (float)this->imu->accelSumCount;
    float vel_acc = accZ_tmp * this->imu->accelVelScale * this->imu->accelTimeSum;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    this->accelAlt += (vel_acc * 0.5f) * dt + this->accelVel * dt;                                         
    this->accelVel += vel_acc;

    // complementary filter for altitude estimation (baro & acc)
    //this->accelAlt = cfilter(this->accelAlt, this->FusedBarosonarAlt, CONFIG_BARO_CF_ALT);

    int32_t estAlt = sonarInRange() ? this->FusedBarosonarAlt : this->accelAlt;

    // reset acceleromter sum
    this->imu->resetAccelSum();

    uint32_t dTime = currentTime - this->previousTime;
    int32_t fusedBaroSonarVel = (this->FusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime;
    lastFusedBarosonarAlt = this->FusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = applyDeadband(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    this->accelVel = cfilter(this->accelVel, fusedBaroSonarVel, CONFIG_BARO_CF_VEL);
    int32_t vel_tmp = lrintf(this->accelVel);

    if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)

        int32_t setVel = 0;

        // Altitude P-Controller
        if (!this->velocityControl) {
            int32_t error = constrain(altHold - estAlt, -500, 500);
            error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - vel_tmp;
        this->altPID = constrain((CONFIG_VEL_P * error / 32), -300, +300);

        // I
        this->errorVelocityI += (CONFIG_VEL_I * error);
        this->errorVelocityI = constrain(this->errorVelocityI, -(8196 * 200), (8196 * 200));
        this->altPID += this->errorVelocityI / 8196;     // I in the range of +/-200

        // D
        this->altPID -= constrain(CONFIG_VEL_D * (accZ_tmp + this->accZ_old) / 512, -150, 150);

    } else {
        this->altPID = 0;
    }

    this->accZ_old = accZ_tmp;

    return estAlt;
}

#ifdef __arm__
} // extern "C"
#endif
