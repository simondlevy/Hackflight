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

            static constexpr float kAxisDeadband = 0.02;

        public:

            Receiver( const MspParser & parser, const uint32_t timestamp_msec)
                : parser_(parser), timestamp_msec_(timestamp_msec) {}

            Receiver() = default;

            static auto ParseByte(
                    const Receiver & rx,
                    const uint8_t byte,
                    const uint32_t time_msec
                    ) -> Receiver
            {
                auto parser = MspParser::Parse(rx.parser_, byte);

                const auto got_new_message =
                    MspParser::GetId(parser) == kMspSetChannels;

                const auto timestamp_msec =
                    got_new_message ? time_msec : rx.timestamp_msec_;

                return Receiver(parser, timestamp_msec);
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

            static auto IsArmingButtonDown(const Receiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 4);
            }

            static auto IsHoverButtonDown(const Receiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 5);
            }

            static auto IsAutopilotButtonDown(const Receiver & rx) -> bool
            {
                return GetSwitchStatus(rx.parser_, 6);
            }

        private:

            MspParser parser_;
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
