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
#include <firmware/debugger.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class EspNowReceiver {

        private:

            static constexpr float kThrottleDownMax = -0.90;

         public:

            EspNowReceiver() = default;

            EspNowReceiver& operator=(
                    const EspNowReceiver& other) = default;

            static auto Update(
                    const EspNowReceiver & rx,
                    const uint8_t byte,
                    const uint32_t time_msec,
                    const bool require_throttle_down_to_arm=true
                    ) -> EspNowReceiver
            {

                const auto is_throttle_down =
                    GetThrottle(rx) < kThrottleDownMax;

                const auto safe_to_arm = require_throttle_down_to_arm ? 
                    is_throttle_down : true;

                printf("safe to arm: %d\n", safe_to_arm);

                /*
                // Push-button arming; ignores startup transient
                const auto did_aux_change_ = tdata.aux_ >= 988 && aux !=
                    tdata.aux_;

                const auto requested_arming = 
                    did_aux_change_ && tdata.data.requested_arming ? false :
                    did_aux_change_ && safe_to_arm ? true :
                    tdata.data.requested_arming;
                    */


               return EspNowReceiver(
                        MspParser::Parse(rx.parser_, byte),
                        MspParser::GetId(rx.parser_) == kMspSetChannels ?  time_msec :
                        rx.time_msec_);
            }

            static auto GetThrottle(const EspNowReceiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 0);
            }

            static auto GetRoll(const EspNowReceiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 1);
            }

            static auto GetPitch(const EspNowReceiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 2);
            }

            static auto GetYaw(const EspNowReceiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 3);
            }

            static auto DidRequestArming(const EspNowReceiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 4);
            }

            static auto DidRequestHover(const EspNowReceiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 5);
            }

            static auto DidRequestAutopilot(const EspNowReceiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 6);
            }

            static auto GetTimestampMsec(const EspNowReceiver & rx) -> uint32_t
            {
                return rx.time_msec_;
            }

        private:

            MspParser parser_;

            uint32_t time_msec_;

            EspNowReceiver(const MspParser & parser, const uint32_t time_msec) :
                parser_(parser), time_msec_(time_msec) {}

            static auto GetAxisValue(
                    const MspParser & parser, const uint8_t index) -> float
            {
                const auto val = MspParser::GetShort(parser, index);

                return 2 * (val / 4095.f ) - 1;
            }

            static auto GetSwitchStatus(
                    const MspParser & parser, const uint8_t index) -> float
            {
                return MspParser::GetShort(parser, index) > 0;
            }
    };
}
