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

namespace hf {

    class Mixer {

        public:

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

            static auto run(const Mixer & mixer, const Setpoint & setpoint) -> Mixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                const auto y = setpoint.yaw;

                const float m1 = t + r * mixer.roll[0] + p * mixer.pitch[0] + y * mixer.yaw[0];
                const float m2 = t + r * mixer.roll[1] + p * mixer.pitch[1] + y * mixer.yaw[1];
                const float m3 = t + r * mixer.roll[2] + p * mixer.pitch[2] + y * mixer.yaw[2];
                const float m4 = t + r * mixer.roll[3] + p * mixer.pitch[3] + y * mixer.yaw[3];

                return Mixer(m1, m2, m3, m4);
            }

    };

}

