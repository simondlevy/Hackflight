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

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <array>

#include <hackflight.h>

namespace hf {

    class MspParser {

        private:

            typedef std::array<uint8_t, 256> buffer_t;

        public:

            MspParser() = default;

            MspParser(
                    const uint8_t state,
                    const buffer_t buffer,
                    const uint8_t expected,
                    const uint8_t received,
                    const uint8_t checksum,
                    const uint8_t index,
                    const uint8_t id)
                :
                    state(state),
                    buffer(buffer),
                    expected(expected),
                    received(received),
                    checksum(checksum),
                    index(index),
                    id(id) {}

            static auto parse(const MspParser & p,
                    const uint8_t b) -> MspParser
            {

                const auto state = 
                    p.state == 0 && b == '$' ? 1 :
                    p.state == 1 && b == 'M' ? 2 :
                    p.state == 1 ? 0 :
                    p.state == 2 ? 3 :
                    p.state == 3 ? 4 :
                    p.state == 4 && p.expected > 0 ? 5 :
                    p.state == 4 ? 6 :
                    p.state == 5 && (p.received >= p.expected) ? 6 :
                    p.state == 6 ? 0 :
                    p.state;

                const auto expected = p.state == 3 ? b : p.expected;

                const auto checksum = p.state == 3 ? b : p.checksum ^ b;

                const auto received = p.state == 5 ? p.received + 1 : 0;

                const auto id = p.state == 4 ? b : p.id;

                const auto index =
                    p.state == 3 ? 0 :
                    p.state == 5 ? p.index + 1 :
                    p.index;

                auto newbuf = p.buffer;
                newbuf[p.index] = b;
                const auto buffer = p.state == 5 ? newbuf : p.buffer;

                return MspParser(state, buffer, expected, received, checksum,
                        index, id);
            }

            static auto getid(const MspParser & p) -> uint8_t
            {
                return p.id;
            }

            static auto getUshort(const MspParser & p,
                    const uint8_t index) -> uint16_t
            {
                const uint8_t offset = 2 * index;
                return (p.buffer[offset+1] << 8) | p.buffer[offset];
            }

        private:

            uint8_t state;
            buffer_t buffer;
            uint8_t expected;
            uint8_t received;
            uint8_t checksum;
            uint8_t index;
            uint8_t id;
    };

}
