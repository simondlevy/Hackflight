/**
 * Class for old-school R/C receiver (throttle must be down to arm)
 *
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

    class TraditionalReceiver {

        private:

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

        public:

            Setpoint axes;
            bool is_armed;
            bool is_throttle_down;
            uint32_t timestamp_msec;

            TraditionalReceiver() = default;

            TraditionalReceiver(
                    const Setpoint & axes,
                    const bool is_armed,
                    const bool is_throttle_down,
                    const uint32_t timestamp_msec,
                    const uint16_t aux)
                :
                    axes(axes),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down),
                    timestamp_msec(timestamp_msec),
                    _aux(aux) {}

            TraditionalReceiver& operator=(
                    const TraditionalReceiver& other) = default;

            static auto update(
                    const TraditionalReceiver & data,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux,
                    const uint32_t msec_curr,
                    const bool require_throttle_down_to_arm=true
                    ) -> TraditionalReceiver
            {
                const auto axes = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                const auto is_throttle_down = axes.thrust <
                    THROTTLE_DOWN_MAX;

                const auto safe_to_arm = require_throttle_down_to_arm ? 
                    is_throttle_down : true;

                // Push-button arming; ignores startup transient
                const auto did_aux_change = data._aux >= 988 && aux !=
                    data._aux;

                const auto is_armed = 
                    did_aux_change && data.is_armed ? false :
                    did_aux_change && safe_to_arm ? true :
                    data.is_armed;

                return TraditionalReceiver(axes, is_armed,
                        is_throttle_down, msec_curr, aux);
            }

        private:

            uint16_t _aux;

            static auto scale(const uint16_t val) -> float
            {
                return 2 * (val - 1500.f) / 1024;
            }

    };
}
