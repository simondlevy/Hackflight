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

#include <firmware/datatypes.hpp>

namespace hf {

    class TraditionalReceiver {

        private:

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

        public:

            ReceiverData data;

            bool is_throttle_down;

            TraditionalReceiver() = default;

            TraditionalReceiver(
                    const Setpoint & setpoint,
                    const bool requested_arming,
                    const uint32_t timestamp_msec,
                    const bool is_throttle_down,
                    const uint16_t aux)
                :
                    data(setpoint, requested_arming, false, timestamp_msec),
                    is_throttle_down(is_throttle_down),
                    aux_(aux) {}

            TraditionalReceiver& operator=(
                    const TraditionalReceiver& other) = default;

            static auto update(
                    const TraditionalReceiver & tdata,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux,
                    const uint32_t msec_curr,
                    const bool require_throttle_down_to_arm=true
                    ) -> TraditionalReceiver
            {
                const auto setpoint = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                const auto is_throttle_down = setpoint.thrust <
                    THROTTLE_DOWN_MAX;

                const auto safe_to_arm = require_throttle_down_to_arm ? 
                    is_throttle_down : true;

                // Push-button arming; ignores startup transient
                const auto didaux__change = tdata.aux_ >= 988 && aux !=
                    tdata.aux_;

                const auto requested_arming = 
                    didaux__change && tdata.data.requested_arming ? false :
                    didaux__change && safe_to_arm ? true :
                    tdata.data.requested_arming;

                return TraditionalReceiver(setpoint, requested_arming,
                        msec_curr, is_throttle_down, aux);
            }

        private:

            uint16_t aux_;

            static auto scale(const uint16_t val) -> float
            {
                return 2 * (val - 1500.f) / 1024;
            }

    };
}
