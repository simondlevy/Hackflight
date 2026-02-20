/*
 *  BetaFlight QuadX motor mixer for Hackflight
 *
 *               4:cw   2:ccw
 *                   \ /
 *                    X 
 *                   / \
 *               3:ccw  1:cw
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */


#pragma once

#include <rotormixer.hpp>

namespace hf {

    class Mixer {

        public:

            static const uint8_t rotorCount = 4;

            static constexpr int8_t roll[4]  = {-1, -1, +1, +1};
            static constexpr int8_t pitch[4] = {+1, -1, +1, -1};
            static constexpr int8_t yaw[4]   = {-1, +1, +1, -1};

            static void mix(const setpoint_t & demands, float motors[])
            {
                RotorMixer::mix(demands, roll, pitch, yaw, 4, motors);
            }
    };

}

