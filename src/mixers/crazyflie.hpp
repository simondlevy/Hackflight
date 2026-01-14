/**
 *  Crazyflie motor mixer for Hackflight
 *
 *               4:cw   1:ccw
 *                   \ /
 *                    X
 *                   / \
 *               3:ccw  2:cw
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <datatypes.h>

class Mixer {

    public:

        static const uint8_t rotorCount = 4;

        static constexpr int8_t roll[rotorCount]  = {-1, -1, +1, +1};
        static constexpr int8_t pitch[rotorCount] = {-1, +1, +1, -1};
        static constexpr int8_t yaw[rotorCount]   = {+1, -1, +1, -1};

        static void mix(const demands_t & demands, float motors[])
        {
            for (uint8_t k=0; k<rotorCount; ++k) {
                motors[k] =
                    demands.thrust +
                    demands.roll*roll[k] +
                    demands.pitch*pitch[k] +
                    demands.yaw*yaw[k];
            }
        }
};

