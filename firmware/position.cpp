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

static const float CONFIG_BARO_CF_ALT = 0.965f;
static const float CONFIG_BARO_CF_VEL = 0.985f;

static const uint8_t CONFIG_ALT_P = 200;
static const uint8_t CONFIG_VEL_P = 200;
static const uint8_t CONFIG_VEL_I = 45;
static const uint8_t CONFIG_VEL_D = 1;

static const bool     CONFIG_ALT_HOLD_FAST_CHANGE       = true;
static const uint16_t CONFIG_ALT_HOLD_THROTTLE_NEUTRAL  = 40;

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


void Position::init(Board * _board, IMU * _imu, Baro * _baro, RC * _rc)
{
    this->board = _board;
    this->imu   = _imu;
    this->baro  = _baro;
    this->rc    = _rc;

    this->altHoldValue = 0;
    this->altHoldMode = false;
    this->estAlt = 0;
    this->altPID = 0;
    this->setVelocity = 0;
    this->velocityControl = false;
    this->errorVelocityI = 0;
    this->initialThrottleHold = 0;
    this->previousT = 0;
    this->accZ_old = 0;
    this->accelVel = 0;
    this->fusedBarosonarAlt = 0;
    this->lastFusedBarosonarAlt = 0;
    this->baroAlt = 0;
    this->baroAltBaseline = 0;
    this->accelAlt = 0;
    this->wasArmed = false;
    this->baroAlt_offset = 0;
    this->sonarTransition = 0;

}

void Position::update(void)
{
    if (this->rc->auxState() > 0) {
        if (!this->altHoldMode) {
            this->altHoldMode = true;
            this->altHoldValue = estAlt;
            this->initialThrottleHold = this->rc->command[THROTTLE];
            this->errorVelocityI = 0;
            this->altPID = 0;
        }
    }
    else 
        this->altHoldMode = false;
}

void Position::computeAltitude(bool armed)
{
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
            this->baroAltBaseline = baroAltRaw;
            accelVel = 0;
            accelAlt = 0;
        }
        this->baroAlt = baroAltRaw - this->baroAltBaseline;
    }
    else {
        this->baroAlt = 0;
    }
    wasArmed = armed;

    // Calculate sonar altitude only if the sonar is facing downwards(<25deg)
    int32_t sonarAlt = -1; // XXX (tiltAngle > 250) ? -1 : sonarAlt * (900.0f - tiltAngle) / 900.0f;

    // Fuse sonarAlt and baroAlt
    if (sonarInRange()) {
        this->baroAlt_offset = this->baroAlt - sonarAlt;
        this->fusedBarosonarAlt = sonarAlt;
    } else {
        this->baroAlt -= this->baroAlt_offset;
        if (sonarAlt > 0) {
            this->sonarTransition = (300 - sonarAlt) / 100.0f;
            this->fusedBarosonarAlt = 
                (int32_t)cfilter((float)sonarAlt, (float)this->baroAlt, (float)this->sonarTransition); 
        }
    }

    // delta acc reading time in seconds
    float dt = this->imu->accelTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    float accZ_tmp = (float)this->imu->accelSum[2] / (float)this->imu->accelSumCount;
    float vel_acc = accZ_tmp * this->imu->accelVelScale * (float)this->imu->accelTimeSum;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    accelAlt += (vel_acc * 0.5f) * dt + accelVel * dt;                                         
    accelVel += vel_acc;

    // complementary filter for altitude estimation (baro & acc)
    accelAlt = cfilter((float)accelAlt, (float)fusedBarosonarAlt, (float)CONFIG_BARO_CF_ALT);

    this->estAlt = sonarInRange() ? fusedBarosonarAlt : (int32_t)accelAlt;

    // reset acceleromter sum
    this->imu->resetAccelSum();

    int32_t fusedBaroSonarVel = (int32_t)((fusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime);
    lastFusedBarosonarAlt = fusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = applyDeadband(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    accelVel = cfilter((float)accelVel, (float)fusedBaroSonarVel, (float)CONFIG_BARO_CF_VEL);
    int32_t vel_tmp = (int32_t)lrintf(accelVel);

    // For now, use baro only for altitude estimate
    this->estAlt = this->baroAlt;

    if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)

        int32_t setVel = this->setVelocity;

        // Altitude P-Controller
        if (!this->velocityControl) {
            int32_t error = constrain(this->altHoldValue - this->estAlt, -500, 500);
            error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - vel_tmp;
        this->altPID = constrain((CONFIG_VEL_P * error / 32), -300, +300);

        // I
        errorVelocityI += (CONFIG_VEL_I * error);
        errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
        this->altPID += errorVelocityI / 8196;     // I in the range of +/-200

        // D
        this->altPID -= constrain(CONFIG_VEL_D * (accZ_tmp + accZ_old) / 512, -150, 150);

    } else {
        this->altPID = 0;
    }

    accZ_old = accZ_tmp;
}

void Position::holdAltitude(void)
{
// For now, support alt-hold in simulation only
#ifdef _SIM
    if (this->altHoldMode) {
        static bool isaltHoldChanged = false;
        if (CONFIG_ALT_HOLD_FAST_CHANGE) {
            // rapid alt changes
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_ALT_HOLD_THROTTLE_NEUTRAL) {
                errorVelocityI = 0;
                isaltHoldChanged = true;
                this->rc->command[THROTTLE] += (this->rc->command[THROTTLE] > this->initialThrottleHold) 
                    ? -CONFIG_ALT_HOLD_THROTTLE_NEUTRAL : CONFIG_ALT_HOLD_THROTTLE_NEUTRAL;
            } else {
                if (isaltHoldChanged) {
                    this->altHoldValue = this->estAlt;
                    isaltHoldChanged = false;
                }
                this->rc->command[THROTTLE] = constrain(this->initialThrottleHold + this->altPID, 
                        CONFIG_PWM_MIN, CONFIG_PWM_MAX);
            }
        } else {
            // slow alt changes
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_ALT_HOLD_THROTTLE_NEUTRAL) {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                this->setVelocity = (this->rc->command[THROTTLE] - this->initialThrottleHold) / 2;
                this->velocityControl = true;
                isaltHoldChanged = true;
            } else if (isaltHoldChanged) {
                this->altHoldValue = this->estAlt;
                this->velocityControl = false;
                isaltHoldChanged = false;
            }
            this->rc->command[THROTTLE] = constrain(this->initialThrottleHold + this->altPID, CONFIG_PWM_MIN, CONFIG_PWM_MAX);
        }
        debug("estAlt: %d    altPID: %d\n", this->estAlt, this->altPID);
    }
#endif // _SIM
}

#ifdef __arm__
} // extern "C"
#endif
