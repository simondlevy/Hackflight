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

                return EspNowReceiver(parser);
            }

            auto IsReady() -> bool
            {
                return MspParser::GetId(parser_) == kMspSetChannels;
            }

            auto GetThrottleValue() -> float
            {
                return GetAxisValue(parser_, 0);
            }

        private:

            MspParser parser_;

            EspNowReceiver(const MspParser & parser) :
                parser_(parser) {}

            static auto GetAxisValue(
                    const MspParser & parser, const uint8_t index) -> float
            {
                const auto val = MspParser::GetShort(parser, index);

                return 2 * (val / 4095.f ) - 1;
            }
    };
}
