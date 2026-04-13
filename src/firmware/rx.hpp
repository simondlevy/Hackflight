/**
 * Copyright (C) 2026 Simon D. Levy
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

#include <datatypes.hpp>

namespace hf {

    class RX {

        public:

            static constexpr uint32_t TIMEOUT_MSEC = 500;

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

            Setpoint axes;
            uint16_t aux;
            bool is_armed;
            bool is_throttle_down;
            uint32_t msec_prev;

            RX() = default;

            RX(
                    const Setpoint & axes,
                    const uint16_t aux,
                    const bool is_armed,
                    const bool is_throttle_down,
                    const uint32_t msec_prev)
                :
                    axes(axes),
                    aux(aux),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down),
                    msec_prev(msec_prev) {}

            RX& operator=(const RX& other) = default;

            static float scale(const uint16_t val)
            {
                return 2 * (val - 1500.f) / 1024;
            }

            static auto update(
                    const RX & rx,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux,
                    const uint32_t msec_curr) -> RX
            {
                const auto axes = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                const auto is_throttle_down = axes.thrust < THROTTLE_DOWN_MAX;

                (void)axes;
                (void)is_throttle_down;

                return rx;
            }

    };
}
