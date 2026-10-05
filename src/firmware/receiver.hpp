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

#include <hackflight.h>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

namespace hf {

    class Receiver {

        private:

            static constexpr float kThrottleDownMax = -0.90;
            static constexpr float kAxisDeadband = 0.02;

        public:

            Receiver(
                    const MspParser & parser,
                    const bool is_down,
                    const bool was_down,
                    const bool armed,
                    const uint32_t timestamp_msec) 
                : parser_(parser),
                is_down_(is_down),
                was_down_(was_down),
                is_armed_(armed),
                timestamp_msec_(timestamp_msec) {}

            Receiver() : was_down_(true) {}

            Receiver& operator=(const Receiver& other) = default;

            static auto ParseByte(
                    const Receiver & rx,
                    const uint8_t byte,
                    const uint32_t time_msec
                    ) -> Receiver
            {
                auto parser = MspParser::Parse(rx.parser_, byte);

                const auto got_new_message =
                    MspParser::GetId(parser) == kMspSetChannels;

                const auto is_down =
                    got_new_message ?
                    MspParser::GetShort(parser, 4) > 0 :
                    rx.is_down_;

                const auto timestamp_msec =
                    got_new_message ? time_msec : rx.timestamp_msec_;

                return Receiver(parser, is_down, rx.was_down_,
                        rx.is_armed_, timestamp_msec);
            }

            static auto CheckArming(const Receiver & rx) -> Receiver
            {
                const auto armed = !rx.is_down_ ? false :

                    rx.is_down_ && !rx.was_down_ &&
                    GetThrottle(rx) < kThrottleDownMax ? true :

                    rx.is_armed_;

                return Receiver(rx.parser_, rx.is_down_, rx.is_down_,
                        armed, rx.timestamp_msec_);
            }

            static auto IsArmed(const Receiver & rx) -> bool
            {
                return rx.is_armed_;
            }

            // Special treatment for throttle for now
            static auto GetThrottle(const Receiver & rx) -> float
            {
                const auto val = MspParser::GetShort(rx.parser_, 0);
                return 2 * (val / 4095.f ) - 1;
            }

            static auto GetRoll(const Receiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 1);
            }

            static auto GetPitch(const Receiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 2);
            }

            static auto GetYaw(const Receiver & rx) -> float
            {
                return GetAxisValue(rx.parser_, 3);
            }

            static auto GetTimestampMsec(const Receiver & rx) -> uint32_t
            {
                return rx.timestamp_msec_;
            }

        private:

            MspParser parser_;
            bool is_down_;
            bool was_down_;
            bool is_armed_;
            uint32_t timestamp_msec_;

            static auto GetAxisValue(
                    const MspParser & parser, const uint8_t index) -> float
            {
                const float val = MspParser::GetShort(parser, index) / 2048.f;

                return fabs(val) < kAxisDeadband ? 0 : val;
            }

            static auto GetSwitchStatus(
                    const MspParser & parser, const uint8_t index) -> float
            {
                return MspParser::GetShort(parser, index) > 0;
            }
    };
}
