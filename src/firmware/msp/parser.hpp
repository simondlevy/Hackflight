/*
   Multiwii Serial Protocol parsing support for Hackflight

   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received_ a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <array>

#include <hackflight.h>

namespace hf {

    class MspParser {

        private:

            typedef std::array<uint8_t, 256> Buffer;

        public:

            MspParser() = default;

            static auto Parse(const MspParser & p,
                    const uint8_t b) -> MspParser
            {

                const auto state_ = 
                    p.state_ == 0 && b == '$' ? 1 :
                    p.state_ == 1 && b == 'M' ? 2 :
                    p.state_ == 1 ? 0 :
                    p.state_ == 2 ? 3 :
                    p.state_ == 3 ? 4 :
                    p.state_ == 4 && p.expected_ > 0 ? 5 :
                    p.state_ == 4 ? 6 :
                    p.state_ == 5 && (p.received_ >= p.expected_) ? 6 :
                    p.state_ == 6 ? 0 :
                    p.state_;

                const auto expected_ = p.state_ == 3 ? b : p.expected_;

                const auto checksum_ = p.state_ == 3 ? b : p.checksum_ ^ b;

                const auto received_ = p.state_ == 5 ? p.received_ + 1 : 0;

                const auto id = p.state_ == 4 ? b : p.id_;

                const auto index_ =
                    p.state_ == 3 ? 0 :
                    p.state_ == 5 ? p.index_ + 1 :
                    p.index_;

                auto newbuf = p.buffer_;
                newbuf[p.index_] = b;
                const auto buffer_ = p.state_ == 5 ? newbuf : p.buffer_;

                return MspParser(state_, buffer_, expected_, received_, checksum_,
                        index_, id);
            }

            static auto GetId(const MspParser & p) -> uint8_t
            {
                return p.id_;
            }

            static auto GetFloat(const MspParser & p,
                    const uint8_t index_) -> float
            {
                const uint8_t offset = 4 * index_;
                uint32_t tmp = (uint32_t) (
                        p.buffer_[offset+3] << 24 |
                        p.buffer_[offset+2] << 16 |
                        p.buffer_[offset+1] << 8 |
                        p.buffer_[offset]);
                float value = 0;
                memcpy(&value, &tmp, 4);
                return value;
            }

        private:

            uint8_t state_;
            Buffer buffer_;
            uint8_t expected_;
            uint8_t received_;
            uint8_t checksum_;
            uint8_t index_;
            uint8_t id_;

            MspParser(
                    const uint8_t state,
                    const Buffer buffer,
                    const uint8_t expected,
                    const uint8_t received,
                    const uint8_t checksum,
                    const uint8_t index,
                    const uint8_t id)
                :
                    state_(state),
                    buffer_(buffer),
                    expected_(expected),
                    received_(received),
                    checksum_(checksum),
                    index_(index),
                    id_(id) {}
    };

}
