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

#include <datatypes.h>

#include <hackflight.hpp>
#include "../constrain.h"
#include "../mixer.hpp"

class FixedPitchMixer : public Mixer {

    public:

        static void fun(
                const demands_t & demands,
                const uint8_t motorCount,
                const Axis3f spins[],
                float motorvals[])
        {
            float mix[Hackflight::MAX_MOTOR_COUNT];

            float mixMax = 0, mixMin = 0;

            for (auto i=0; i<motorCount; i++) {

                mix[i] =
                    demands.roll  * spins[i].x +
                    demands.pitch * spins[i].y +
                    demands.yaw   * spins[i].z;

                if (mix[i] > mixMax) {
                    mixMax = mix[i];
                } else if (mix[i] < mixMin) {
                    mixMin = mix[i];
                }
                mix[i] = mix[i];
            }

            float motorRange = mixMax - mixMin;

            float throttle = demands.thrust;

            if (motorRange > 1.0f) {
                for (auto i=0; i<motorCount; i++) {
                    mix[i] /= motorRange;
                }
            } else {
                if (demands.thrust > 0.5f) {
                    throttle = constrain_f(throttle, -mixMin, 1.0f - mixMax);
                }
            }

            for (auto i=0; i<motorCount; i++) {

                motorvals[i] = mix[i] + throttle;
            }

        } // run

}; // class FixedPitchMixer
