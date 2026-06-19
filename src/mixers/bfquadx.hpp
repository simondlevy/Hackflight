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

#include <datatypes.hpp>

namespace hf {

    class Mixer {

        public:

            uint8_t motorcount = 4;

            int8_t roll[4]  = {-1, -1, +1, +1};
            int8_t pitch[4] = {+1, -1, +1, -1};
            int8_t yaw[4]   = {-1, +1, +1, -1};

            float motorvals[4];

            Mixer() = default;

            Mixer(
                    const float m1,
                    const float m2,
                    const float m3,
                    const float m4)
            {
                motorvals[0] = m1;
                motorvals[1] = m2;
                motorvals[2] = m3;
                motorvals[3] = m4;
            }

            Mixer& operator=(const Mixer& other) = default;

            static auto Run(const Setpoint & setpoint)-> Mixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                const auto y = setpoint.yaw;

                const float m1 = t - r + p - y;
                const float m2 = t - r - p + y;
                const float m3 = t + r + p + y;
                const float m4 = t + r - p - y;

                return Mixer(m1, m2, m3, m4);
            }
    };


    float * BFQuadX(const Setpoint & setpoint)
    {
        const auto t = setpoint.thrust;
        const auto r = setpoint.roll;
        const auto p = setpoint.pitch;
        const auto y = setpoint.yaw;

        static float motors[4];

        motors[0] = t - r + p - y;
        motors[1] = t - r - p + y;
        motors[2] = t + r + p + y;
        motors[3] = t + r - p - y;

        return motors;
    }

} // namespace hf
