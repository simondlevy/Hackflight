#pragma once

/*
   Mixer values for quad-X frames using Betaflight motor layout:

   4cw   2ccw
   \ /
   ^
   / \
   3ccw  1cw

   Copyright (C) 2022 Simon D. Levy

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

#include "logic/core/axes.h"
#include "logic/core/mixers/fixedpitch.h"
#include "logic/core/motors/quad.h"

class QuadXbfMixer {

    private:

        static auto fun(const Demands & demands) -> Motors
        {
            Axes SPINS[4] = {
                //  rol   pit    yaw
                Axes( -1.0f, +1.0f, -1.0f ), // REAR_R
                Axes( -1.0f, -1.0f, +1.0f ), // FRONT_R
                Axes( +1.0f, +1.0f, +1.0f ), // REAR_L
                Axes( +1.0f, -1.0f, -1.0f ), // FRONT_L
            };

            float motorvals[4];

            FixedPitchMixer::fun(demands, 4, SPINS, motorvals);

            return QuadMotors::make(motorvals);
        }

    public:

        static Mixer make(void)
        {
            return Mixer(4, fun);
        }
};
