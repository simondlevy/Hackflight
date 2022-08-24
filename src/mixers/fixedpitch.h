/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along
   with Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "motor_device.h"
#include "datatypes.h"

class FixedPitchMixer : public Mixer {

    protected:

        FixedPitchMixer(uint8_t motorCount) 
            : Mixer(motorCount)
        {
        }

        virtual axes_t getSpin(uint8_t k) = 0;

        virtual void run(demands_t * demands, float * motorvals) override
        {
            float mix[MAX_SUPPORTED_MOTORS];

            float mixMax = 0, mixMin = 0;

            for (int i = 0; i < m_motorCount; i++) {

                mix[i] =
                    demands->roll  * getSpin(i).x +
                    demands->pitch * getSpin(i).y +
                    demands->yaw   * getSpin(i).z;

                if (mix[i] > mixMax) {
                    mixMax = mix[i];
                } else if (mix[i] < mixMin) {
                    mixMin = mix[i];
                }
                mix[i] = mix[i];
            }

            float motorRange = mixMax - mixMin;

            float throttle = demands->throttle;

            if (motorRange > 1.0f) {
                for (int i = 0; i < m_motorCount; i++) {
                    mix[i] /= motorRange;
                }
            } else {
                if (demands->throttle > 0.5f) {
                    throttle = constrain_f(throttle, -mixMin, 1.0f - mixMax);
                }
            }

            for (int i = 0; i < m_motorCount; i++) {

                motorvals[i] = mix[i] + throttle;
            }

        } // run

}; // class FixedPitchMixer
