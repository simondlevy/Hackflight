/**
 * Class for mocking up old-school R/C receiver with ESP-NOW
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
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class EspNowReceiver {

        public:

            EspNowReceiver() = default;

            EspNowReceiver& operator=(
                    const EspNowReceiver& other) = default;

            static auto Update(
                    const EspNowReceiver & tdata,
                    const uint8_t byte
                    ) -> EspNowReceiver
            {
                return tdata;
            }

         private:

            MspParser parser_;

#if 0
        private:

            static constexpr float kThrottleDownMax = -0.95;

        public:

            ReceiverData data;

            bool is_throttle_down;

            EspNowReceiver(
                    const Setpoint & setpoint,
                    const bool requested_arming,
                    const uint32_t timestamp_msec,
                    const bool is_throttle_down,
                    const uint16_t aux)
                :
                    data(setpoint, requested_arming, false, timestamp_msec),
                    is_throttle_down(is_throttle_down),
                    aux_(aux) {}

            static auto Update(
                    const EspNowReceiver & tdata,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux,
                    const uint32_t msec_curr,
                    const bool require_throttle_down_to_arm=true
                    ) -> EspNowReceiver
            {
                const auto setpoint = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                const auto is_throttle_down = setpoint.thrust <
                    kThrottleDownMax;

                const auto safe_to_arm = require_throttle_down_to_arm ? 
                    is_throttle_down : true;

                // Push-button arming; ignores startup transient
                const auto didaux__change = tdata.aux_ >= 988 && aux !=
                    tdata.aux_;

                const auto requested_arming = 
                    didaux__change && tdata.data.requested_arming ? false :
                    didaux__change && safe_to_arm ? true :
                    tdata.data.requested_arming;

                return EspNowReceiver(setpoint, requested_arming,
                        msec_curr, is_throttle_down, aux);
            }

        private:

            uint16_t aux_;

            static auto scale(const uint16_t val) -> float
            {
                return 2 * (val - 1500.f) / 1024;
            }
#endif

    };
}
