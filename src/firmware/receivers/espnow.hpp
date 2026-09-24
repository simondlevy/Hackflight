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

        private:

            static constexpr float kThrottleDownMax = -0.95;

        public:

            ReceiverData data;

            bool is_throttle_down;

            EspNowReceiver() = default;

            EspNowReceiver& operator=(
                    const EspNowReceiver& other) = default;

            static auto Update(
                    const EspNowReceiver & rx,
                    const uint8_t byte
                    ) -> EspNowReceiver
            {
                const auto parser = MspParser::Parse(rx.parser_, byte);

                if (MspParser::GetId(parser) == kMspSetChannels) {

                    const auto thr = GetAxisValue(parser, 0);
                    const auto rol = GetAxisValue(parser, 1);
                    const auto pit = GetAxisValue(parser, 2);
                    const auto yaw = GetAxisValue(parser, 3);

                    const auto arm = MspParser::GetShort(parser, 4);
                    const auto hov = MspParser::GetShort(parser, 5);
                    const auto aut = MspParser::GetShort(parser, 6);

                    (void)thr;
                    (void)rol;
                    (void)pit;
                    (void)yaw;
                    (void)arm;
                    (void)hov;
                    (void)aut;
                }

                return rx;
            }

        private:

            MspParser parser_;

            static auto GetAxisValue(
                    const MspParser & parser, const uint8_t index) -> float
            {
                const auto val = MspParser::GetShort(parser, index);

                return 2 * (val / 4095.f ) - 1;
            }

#if 0
        public:

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
                    const EspNowReceiver & rx,
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
                const auto didaux__change = rx.aux_ >= 988 && aux !=
                    rx.aux_;

                const auto requested_arming = 
                    didaux__change && rx.data.requested_arming ? false :
                    didaux__change && safe_to_arm ? true :
                    rx.data.requested_arming;

                return EspNowReceiver(setpoint, requested_arming,
                        msec_curr, is_throttle_down, aux);
            }

        private:

            uint16_t aux_;
            }
#endif

    };
}
