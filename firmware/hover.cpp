/*
   hover.cpp : PID-based hover-in-place class implementation

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

void Hover::init(RC * _rc, Position * _position)
{
    this->rc = _rc;
    this->position = _position;

    this->accelZ_prev = 0;
    this->altHoldValue = 0;
    this->altHoldMode = false;
    this->altPID = 0;
    this->errorVelocityI = 0;
    this->initialThrottleHold = 0;
    this->setVelocity = 0;
    this->velocityControl = false;
}

void Hover::checkSwitch(void)
{
    if (this->rc->auxState() > 0) {
        if (!this->altHoldMode) {
            this->altHoldMode = true;
            this->altHoldValue = this->position->estAlt;
            this->initialThrottleHold = this->rc->command[THROTTLE];
            this->errorVelocityI = 0;
            this->altPID = 0;
        }
    }
    else 
        this->altHoldMode = false;
}

void Hover::updatePid(void)
{
    // only calculate pid if the copter's thrust is facing downwards(<80deg)    
    if (this->position->tiltAngle < 800) { 

        int32_t setVel = this->setVelocity;

        // Altitude P-Controller
        if (!this->velocityControl) {
            int32_t error = constrain(this->altHoldValue - this->position->estAlt, -500, 500);
            error = deadbandFilter(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_HOVER_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - (int32_t)lrintf(this->position->accelVel);
        this->altPID = constrain((CONFIG_HOVER_VEL_P * error / 32), -300, +300);

        // I
        errorVelocityI += (CONFIG_HOVER_VEL_I * error);
        errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
        this->altPID += errorVelocityI / 8196;     // I in the range of +/-200

        // D
        this->altPID -= constrain(CONFIG_HOVER_VEL_D * (this->position->accelZ + this->accelZ_prev) / 512, -150, 150);

    } else 
        this->altPID = 0;

    // Track accelerometer Z
    this->accelZ_prev = this->position->accelZ;
}

void Hover::holdAltitude(void)
{
    // For now, support alt-hold in simulation only
#ifdef _SIM
    if (this->altHoldMode) {
        static bool isaltHoldChanged = false;
        if (CONFIG_HOVER_ALT_HOLD_FAST_CHANGE) {
            // rapid alt changes
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL) {
                errorVelocityI = 0;
                isaltHoldChanged = true;
                this->rc->command[THROTTLE] += (this->rc->command[THROTTLE] > this->initialThrottleHold) 
                    ? -CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL : CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL;
            } else {
                if (isaltHoldChanged) {
                    this->altHoldValue = this->position->estAlt;
                    isaltHoldChanged = false;
                }
                this->rc->command[THROTTLE] = constrain(this->initialThrottleHold + this->altPID, 
                        CONFIG_PWM_MIN, CONFIG_PWM_MAX);
            }
        } else {
            // slow alt changes
            if (abs(this->rc->command[THROTTLE] - this->initialThrottleHold) > CONFIG_HOVER_ALT_HOLD_THROTTLE_NEUTRAL) {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                this->setVelocity = (this->rc->command[THROTTLE] - this->initialThrottleHold) / 2;
                this->velocityControl = true;
                isaltHoldChanged = true;
            } else if (isaltHoldChanged) {
                this->altHoldValue = this->position->estAlt;
                this->velocityControl = false;
                isaltHoldChanged = false;
            }
            this->rc->command[THROTTLE] = constrain(this->initialThrottleHold + this->altPID, CONFIG_PWM_MIN, CONFIG_PWM_MAX);
        }
    }
#endif // _SIM
}

#ifdef __arm__
} // extern "C"
#endif
