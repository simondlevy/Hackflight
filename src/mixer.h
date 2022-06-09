/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdbool.h>

#include "failsafe.h"
#include "maths.h"
#include "motor.h"

#include "mixers/quadxbf.h"

static const float    PID_MIXER_SCALING = 1000.0;
static const uint16_t PIDSUM_LIMIT_YAW  = 400;
static const uint16_t PIDSUM_LIMIT      = 500;

#if defined(__cplusplus)
extern "C" {
#endif

static void mixerRun(demands_t * demands, float * motors)
{
    axes_t * activeMixer = mixerQuadXBF;

    // Calculate and Limit the PID sum
    float roll = 
        constrain_f(demands->roll, -PIDSUM_LIMIT, PIDSUM_LIMIT) / PID_MIXER_SCALING;
    float pitch =
        constrain_f(demands->pitch, -PIDSUM_LIMIT, PIDSUM_LIMIT) / PID_MIXER_SCALING;
    float yaw = 
        -constrain_f(demands->yaw, -PIDSUM_LIMIT_YAW, PIDSUM_LIMIT_YAW) / PID_MIXER_SCALING;

    // reduce throttle to offset additional motor output
    float throttle = demands->throttle;

    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < 4; i++) {

        float mix =
            roll * activeMixer[i].x + pitch * activeMixer[i].y + yaw * activeMixer[i].z;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    float motorRange = motorMixMax - motorMixMin;

    if (motorRange > 1.0f) {
        for (int i = 0; i < 4; i++) {
            motorMix[i] /= motorRange;
        }
    } else {
        if (throttle > 0.5f) {
            throttle = constrain_f(throttle, -motorMixMin, 1.0f - motorMixMax);
        }
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < 4; i++) {
        float motorOutput = motorMix[i] + throttle;
        motorOutput = motorValueLow() + (motorValueHigh() - motorValueLow()) * motorOutput;

        if (failsafeIsActive()) {
            if (motorIsProtocolDshot()) {
                // Prevent getting into special reserved range
                motorOutput = (motorOutput < motorValueLow()) ?
                    motorValueDisarmed() :
                    motorOutput; 
            }
            motorOutput = constrain_f(motorOutput, motorValueDisarmed(), motorValueHigh());
        } else {
            motorOutput = constrain_f(motorOutput, motorValueLow(), motorValueHigh());
        }
        motors[i] = motorOutput;
    }
}

#if defined(__cplusplus)
}
#endif
