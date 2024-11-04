/*
 *  BetaFlight QuadX motor mixer for Hackflight
 *
 *               4:cw   2:ccw
 *                   \ /
 *                    X 
 *                   / \
 *               3:ccw   1:cw
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

#include <hackflight.hpp>
#include <mixer.hpp>

namespace hf {

    class BfQuadXMixer : public Mixer {

        public:

            void run(const demands_t & demands, float * motors)
            {
                const auto t = demands.thrust;
                const auto r = demands.roll;
                const auto p = demands.pitch;
                const auto y = demands.yaw;

                motors[0] = t - r + p  - y;
                motors[1] = t - r - p  + y;
                motors[2] = t + r + p  + y;
                motors[3] = t + r - p  - y;
            }

            float roll(const float * motors)
            {
                return (motors[2] + motors[3]) - (motors[0] + motors[1]);
            }

            float pitch(const float * motors)
            {
                return (motors[0] + motors[1]) - (motors[2] + motors[3]);
            }

            float yaw(const float * motors)
            {
                return (motors[1] + motors[2]) - (motors[0] + motors[3]);
            }
    };

}
