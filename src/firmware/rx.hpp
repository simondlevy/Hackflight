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

        private:

            static constexpr uint32_t TIMEOUT_MSEC = 500;

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

        public:

            Setpoint axes;
            bool is_armed;
            bool is_throttle_down;

            RX() = default;

            RX(
                    const Setpoint & axes,
                    const bool is_armed,
                    const bool is_throttle_down,
                    const uint16_t aux,
                    const uint32_t msec_prev)
                :
                    axes(axes),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down),
                    aux(aux),
                    msec_prev(msec_prev) {}

            RX& operator=(const RX& other) = default;

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

                // Push-button arming; ignores startup transient
                const auto did_aux_change = rx.aux >= 988 && aux != rx.aux;

                const auto is_armed = 
                    did_aux_change && rx.is_armed ? false :
                    did_aux_change && rx.is_throttle_down ? true :
                    rx.is_armed;

                return RX(axes, is_armed, is_throttle_down, aux, msec_curr);
            }

            static auto checkTimeout(const RX & rx,
                    const uint32_t msec_curr) -> RX
            {
                const auto timed_out = 
                    rx.msec_prev > 0 &&
                    msec_curr > rx.msec_prev &&
                    msec_curr - rx.msec_prev > TIMEOUT_MSEC;

                const auto is_armed = timed_out ? false : rx.is_armed;

                return RX(rx.axes, rx.aux, is_armed, rx.is_throttle_down,
                        rx.msec_prev);
            } 

        private:

            uint16_t aux;
            uint32_t msec_prev;

            static float scale(const uint16_t val)
            {
                return 2 * (val - 1500.f) / 1024;
            }

    };
}
