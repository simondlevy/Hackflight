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
            float aux;
            bool is_armed;
            bool is_throttle_down;

            RX() = default;

            RX(
                    const Setpoint & axes,
                    const float aux,
                    const bool is_armed,
                    const bool is_throttle_down)
                :
                    axes(axes),
                    aux(aux),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down) {}

            RX& operator=(const RX& other) = default;

            static float scale(const uint16_t val)
            {
                return 2 * (val - 1500.f) / 1024;
            }

            static auto read(const RX & rx, const uint32_t msec_curr) -> RX
            {
                const auto is_throttle_down = rx.axes.thrust < THROTTLE_DOWN_MAX;
#if 0

                // Check failsafe via RX timeout
                if (_last_rx_msec > 0 &&
                        msec_curr > _last_rx_msec &&
                        msec_curr - _last_rx_msec > RX::TIMEOUT_MSEC) {
                    _rx.is_armed = false;
                }

                // Push-button arming
                static float _chan5_prev;
                const auto chan5_curr = _rx.aux;
                if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
                    _rx.is_armed =
                        _rx.is_armed ? false :
                        _rx.is_throttle_down ? true :
                        _rx.is_armed;
                }
                _chan5_prev = chan5_curr;
#endif

                (void)is_throttle_down;

                return rx;
            }

    };
}
