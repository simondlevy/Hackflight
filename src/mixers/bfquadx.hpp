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

            virtual uint8_t rotorCount() override
            {
                return 4;
            }

            virtual int8_t roll(const uint8_t index) override
            {
                static constexpr int8_t r[4] = {-1, -1, +1, +1};

                return r[index];
            }

            virtual int8_t pitch(const uint8_t index) override
            {
                static constexpr int8_t p[4] = {+1, -1, +1, -1};

                return p[index];
            }

            virtual int8_t yaw(const uint8_t index) override
            {
                static constexpr int8_t y[4] = {+1, -1, -1, +1}; 

                return y[index];
            }

           
    };
}
