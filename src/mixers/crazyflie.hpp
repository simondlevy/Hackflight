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

#include <rotormixer.hpp>

namespace hf {

    class Mixer {

        public:

            static const uint8_t rotorCount = 4;

            static constexpr int8_t ROLL[4]  = {-1, -1, +1, +1};
            static constexpr int8_t PITCH[4] = {-1, +1, +1, -1};
            static constexpr int8_t YAW[4]   = {+1, -1, +1, -1};

            float motorvals[4];

            int8_t roll[4];
            int8_t pitch[4];
            int8_t yaw[4];

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

                roll[0] = ROLL[0];
                roll[1] = ROLL[1];
                roll[2] = ROLL[2];
                roll[3] = ROLL[3];

                pitch[0] = PITCH[0];
                pitch[1] = PITCH[1];
                pitch[2] = PITCH[2];
                pitch[3] = PITCH[3];

                yaw[0] = YAW[0];
                yaw[1] = YAW[1];
                yaw[2] = YAW[2];
                yaw[3] = YAW[3];
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

            static void mix(
                    const Setpoint & setpoint,
                    const int8_t * roll,
                    const int8_t * pitch,
                    const int8_t * yaw,
                    const int8_t count,
                    float motors[])
            {
                for (uint8_t k=0; k<count; ++k) {
                    motors[k] =
                        setpoint.thrust +
                        setpoint.roll * roll[k] +
                        setpoint.pitch * pitch[k] +
                        setpoint.yaw * yaw[k];
                }
            }

            static void mix(const Setpoint & setpoint, float motors[])
            {
                mix(setpoint, ROLL, PITCH, YAW, 4, motors);
            }
    };
}
