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

#pragma once

#include <stdbool.h>

#include "datatypes.h"
#include "debug.h"
#include "failsafe.h"
#include "maths.h"
#include "motor.h"

#if defined(__cplusplus)
extern "C" {
#endif

    static void fixedPitchMix(
            float throttle,
            float roll,
            float pitch,
            float yaw,
            axes_t * axes,
            uint8_t motorCount,
            float * motors)
    {
        float mix[MAX_SUPPORTED_MOTORS];

        float mixMax = 0, mixMin = 0;

        for (int i = 0; i < motorCount; i++) {

            mix[i] = roll * axes[i].x + pitch * axes[i].y + yaw * axes[i].z;

            if (mix[i] > mixMax) {
                mixMax = mix[i];
            } else if (mix[i] < mixMin) {
                mixMin = mix[i];
            }
            mix[i] = mix[i];
        }

        float motorRange = mixMax - mixMin;

        if (motorRange > 1.0f) {
            for (int i = 0; i < motorCount; i++) {
                mix[i] /= motorRange;
            }
        } else {
            if (throttle > 0.5f) {
                throttle = constrain_f(throttle, -mixMin, 1.0f - mixMax);
            }
        }

        // Now add in the desired throttle, but keep in a range that doesn't
        // clip adjusted roll/pitch/yaw. This could move throttle down, but
        // also up for those low throttle flips.
        for (int i = 0; i < motorCount; i++) {
            motors[i] = mix[i] + throttle;
        }
    }

#if defined(__cplusplus)
}
#endif
