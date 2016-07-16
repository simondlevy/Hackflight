/*
   navigate.cpp : Navigation code

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
#include "pidvals.hpp"

static const bool     CONFIG_HOVER_ALT_HOLD_FAST_CHANGE       = true;
static const uint16_t CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL  = 40;

static const float CONFIG_BARO_CF_ALT = 0.965f;
static const float CONFIG_BARO_CF_VEL = 0.985f;

static bool sonarInRange(void)
{
    return false; // XXX
}


void Navigation::init(Board * _board, IMU * _imu, Baro * _baro, RC * _rc)
{
    this->board = _board;
    this->imu   = _imu;
    this->baro  = _baro;
    this->rc = _rc;

    this->accelAlt = 0;
    this->accelVel = 0;
    this->accelZ = 0;
    this->accelZ_prev = 0;
    this->altHoldValue = 0;
    this->altHoldMode = false;
    this->altPID = 0;
    this->baroAlt = 0;
    this->baroAlt_offset = 0;
    this->baroAltBaseline = 0;
    this->errorVerticalVelocityI = 0;
    this->estAlt = 0;
    this->flightMode = MODE_NORMAL;
    this->fusedBarosonarAlt = 0;
    this->initialThrottleHold = 0;
    this->lastFusedBarosonarAlt = 0;
    this->previousT = 0;
    this->setVerticalVelocity = 0;
    this->sonarTransition = 0;
    this->tiltAngle = 0;
    this->vario = 0;
    this->verticalVelocityControl = false;
    this->verticalVelocity = 0;
    this->wasArmed = false;
}

void Navigation::checkSwitch(void)
{
    if (this->rc->auxState() > 0) {
        if (!this->altHoldMode) {
            this->altHoldMode = true;
            this->altHoldValue = this->estAlt;
            this->initialThrottleHold = this->rc->command[THROTTLE];
            this->errorVerticalVelocityI = 0;
            this->altPID = 0;
        }
    }
    else 
        this->altHoldMode = false;
}

void Navigation::updateAltitudePid(bool armed)
{
    this->tiltAngle = max(abs(this->imu->angle[ROLL]), abs(this->imu->angle[PITCH]));

    uint32_t currentT = this->board->getMicros();
    uint32_t dTime = currentT - previousT;

    if (dTime < CONFIG_ALTITUDE_UPDATE_MSEC*1000) 
        return;

    previousT = currentT;

    int32_t baroAltRaw = this->baro->getAltitude();

    // Grab baro baseline on arming
    if (armed) {
        if (!wasArmed) {
            this->baroAltBaseline = baroAltRaw;
            this->accelVel = 0;
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
                (int32_t)complementaryFilter((float)sonarAlt, (float)this->baroAlt, (float)this->sonarTransition); 
        }
    }

    // delta acc reading time in seconds
    float dt = this->imu->accelTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    this->accelZ = (float)this->imu->accelSum[2] / (float)this->imu->accelSumCount;
    float vel_acc = this->accelZ * this->imu->accelVelScale * (float)this->imu->accelTimeSum;

    // integrate accelerometer velocity to get vertical velocity
    this->verticalVelocity += vel_acc;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    this->accelAlt += (vel_acc * 0.5f) * dt + this->accelVel * dt;                                         
    this->accelVel += vel_acc;

    // complementary filter for altitude estimation (baro & acc)
    accelAlt = complementaryFilter((float)accelAlt, (float)fusedBarosonarAlt, (float)CONFIG_BARO_CF_ALT);

    //this->estAlt = sonarInRange() ? fusedBarosonarAlt : (int32_t)accelAlt;

    // reset acceleromter sum
    this->imu->resetAccelSum();

    int32_t fusedBaroSonarVel = (int32_t)((fusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime);
    lastFusedBarosonarAlt = fusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = deadbandFilter(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    this->accelVel = complementaryFilter((float)accelVel, (float)fusedBaroSonarVel, (float)CONFIG_BARO_CF_VEL);

    // For now, use baro only for altitude estimate
    this->estAlt = this->baroAlt;

    // only calculate pid if the copter's thrust is facing downwards(<80deg)    
    if (this->tiltAngle < 800) { 

        int32_t setVel = this->setVerticalVelocity;

        // Altitude P-Controller
        if (!this->verticalVelocityControl) {
            int32_t error = constrain(this->altHoldValue - this->estAlt, -500, 500);
            error = deadbandFilter(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_HOVER_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - (int32_t)lrintf(this->accelVel);
        this->altPID = constrain((CONFIG_HOVER_VEL_P * error / 32), -300, +300);

        // I
        errorVerticalVelocityI += (CONFIG_HOVER_VEL_I * error);
        errorVerticalVelocityI = constrain(errorVerticalVelocityI, -(8196 * 200), (8196 * 200));
        this->altPID += errorVerticalVelocityI / 8196;     // I in the range of +/-200

        // D
        this->altPID -= constrain(CONFIG_HOVER_VEL_D * (this->accelZ + this->accelZ_prev) / 512, -150, 150);

    } else 
        this->altPID = 0;

    // Track accelerometer Z
    this->accelZ_prev = this->accelZ;
}


void Navigation::holdAltitude(void)
{
    // For now, support alt-hold in simulation only
#ifdef _SIM
    if (this->altHoldMode) {
        static bool isaltHoldChanged = false;
        if (CONFIG_HOVER_ALT_HOLD_FAST_CHANGE) {
            // rapid alt changes
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL) {
                errorVerticalVelocityI = 0;
                isaltHoldChanged = true;
                this->rc->command[THROTTLE] += (this->rc->command[THROTTLE] > this->initialThrottleHold) 
                    ? -CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL : CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL;
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
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL) {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                this->setVerticalVelocity = (this->rc->command[THROTTLE] - this->initialThrottleHold) / 2;
                this->verticalVelocityControl = true;
                isaltHoldChanged = true;
            } else if (isaltHoldChanged) {
                this->altHoldValue = this->estAlt;
                this->verticalVelocityControl = false;
                isaltHoldChanged = false;
            }
            this->rc->command[THROTTLE] = constrain(this->initialThrottleHold + this->altPID, CONFIG_PWM_MIN, CONFIG_PWM_MAX);
        }
    }
#endif // _SIM
} // updatePid


#ifdef __arm__
} // extern "C"
#endif
