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

                const auto expected = p.state == 3 ? b : p.state;

                const auto checksum = p.state == 3 ? b : p.checksum ^ b;

                const auto received = p.state == 5 ? p.received + 1 : 0;

                const auto id = p.state == 6 && p.checksum == b ? p.id : 0;

                const auto index =
                    p.state == 3 ? 0 :
                    p.state == 5 ? p.index + 1 :
                    p.index;

                auto newbuf = p.buffer;
                newbuf[p.index] = b;
                const auto buffer = p.state == 5 ? newbuf : p.buffer;

                return MspParser(state, buffer, expected, received, checksum, index, id);

            }

            /**
             * Returns message ID or 0 for not  ready
             */
            uint8_t parse(const uint8_t byte)
            {
                uint8_t result = 0;

                //printf("byte=%03d state=%d\n", byte, state);

                switch (state) {

                    case 0:
                        if (byte == '$') {  // $
                            state++;
                        }
                        break;

                    case 1:
                        if (byte == 'M') { // M
                            state++;
                        }
                        else {  // restart and try again
                            state = 0;
                        }
                        break;

                    case 2:
                        state++;
                        break;

                    case 3:
                        expected = byte;
                        checksum = byte;
                        index = 0;
                        state++;
                        break;

                    case 4:
                        id = byte;
                        received = 0;
                        checksum ^= byte;
                        if (expected > 0) {
                            // process payload
                            state++;
                        }
                        else {
                            // no payload
                            state += 2;
                        }
                        break;

                    case 5:
                        buffer[index++] = byte;
                        checksum ^= byte;
                        received++;
                        if (received >= expected) {
                            state++;
                        }
                        break;

                    case 6:

                        if (checksum == byte) {
                            result = id;
                        }
                        state = 0;
                        break;
                }

                return result;
            }

            float getFloat(const uint8_t index)
            {
                const uint8_t offset = 4 * index;
                uint32_t tmp = (uint32_t) (
                        buffer[offset+3] << 24 |
                        buffer[offset+2] << 16 |
                        buffer[offset+1] << 8 |
                        buffer[offset]);
                float value = 0;
                memcpy(&value, &tmp, 4);
                return value;
            }

            uint16_t getShort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                int16_t value = (buffer[offset+1] << 8) | buffer[offset];
                return value;
            }

            uint16_t getUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (buffer[offset+1] << 8) | buffer[offset];
                return value;
            }

            uint8_t getByte(const uint8_t index)
            {
                return buffer[index];
            }

            uint8_t getPayload(uint8_t * payload)
            {
                payload[0] = 36;
                payload[1] = 77;
                payload[2] = 62;
                payload[3] = expected;
                payload[4] = id;

                for (uint8_t k=0; k<expected; ++k) {
                    payload[k+5] = buffer[k];
                }

                payload[5 + expected] = checksum;

                return expected + 6;
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
