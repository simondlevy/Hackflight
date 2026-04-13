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

            typedef std::array<uint8_t, 256> message_buffer_t;

        public:

            MspParser() = default;

            MspParser(
                    const uint8_t state,
                    const message_buffer_t message_buffer,
                    const uint8_t message_expected,
                    const uint8_t message_received,
                    const uint8_t message_checksum,
                    const uint8_t message_index,
                    const uint8_t message_id)
                :
                    _state(state),
                    _buffer(message_buffer),
                    _expected(message_expected),
                    _received(message_received),
                    _checksum(message_checksum),
                    _index(message_index),
                    _id(message_id) {}

            MspParser(const MspParser & parser, const uint8_t state)
                :
                    _state(state),
                    _buffer(parser._buffer),
                    _expected(parser._expected),
                    _received(parser._received),
                    _checksum(parser._checksum),
                    _index(parser._index),
                    _id(parser._id) {}

            static auto parse(const MspParser & parser,
                    const uint8_t byte) -> MspParser
            {
                const auto checksum = parser._checksum ^ byte;

                const auto state = 
                    parser._state == 0 && byte == '$' ? 1 :
                    parser._state == 1 && byte == 'M' ? 2 :
                    parser._state == 1 ? 0 :
                    parser._state == 2 ? 3 :
                    parser._state == 3 ? 4 :
                    parser._state == 4 && parser._expected > 0 ? 5 :
                    parser._state == 4 ? 6 :
                    parser._state == 5 && (parser._received >=
                            parser._expected) ? 6 :
                    parser._state == 6 ? 0 :
                    parser._state;

                (void)checksum;
                (void)state;

                return parser;

            }

            /**
             * Returns message ID or 0 for not  ready
             */
            uint8_t parse(const uint8_t byte)
            {
                uint8_t result = 0;

                //printf("byte=%03d state=%d\n", byte, _state);

                switch (_state) {

                    case 0:
                        if (byte == '$') {  // $
                            _state++;
                        }
                        break;

                    case 1:
                        if (byte == 'M') { // M
                            _state++;
                        }
                        else {  // restart and try again
                            _state = 0;
                        }
                        break;

                    case 2:
                        _state++;
                        break;

                    case 3:
                        _expected = byte;
                        _checksum = byte;
                        _index = 0;
                        _state++;
                        break;

                    case 4:
                        _id = byte;
                        _received = 0;
                        _checksum ^= byte;
                        if (_expected > 0) {
                            // process payload
                            _state++;
                        }
                        else {
                            // no payload
                            _state += 2;
                        }
                        break;

                    case 5:
                        _buffer[_index++] = byte;
                        _checksum ^= byte;
                        _received++;
                        if (_received >= _expected) {
                            _state++;
                        }
                        break;

                    case 6:

                        if (_checksum == byte) {
                            result = _id;
                        }
                        _state = 0;
                        break;
                }

                return result;
            }

            float getFloat(const uint8_t index)
            {
                const uint8_t offset = 4 * index;
                uint32_t tmp = (uint32_t) (
                        _buffer[offset+3] << 24 |
                        _buffer[offset+2] << 16 |
                        _buffer[offset+1] << 8 |
                        _buffer[offset]);
                float value = 0;
                memcpy(&value, &tmp, 4);
                return value;
            }

            uint16_t getShort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                int16_t value = (_buffer[offset+1] << 8) | _buffer[offset];
                return value;
            }

            uint16_t getUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (_buffer[offset+1] << 8) | _buffer[offset];
                return value;
            }

            uint8_t getByte(const uint8_t index)
            {
                return _buffer[index];
            }

            uint8_t getPayload(uint8_t * payload)
            {
                payload[0] = 36;
                payload[1] = 77;
                payload[2] = 62;
                payload[3] = _expected;
                payload[4] = _id;

                for (uint8_t k=0; k<_expected; ++k) {
                    payload[k+5] = _buffer[k];
                }

                payload[5 + _expected] = _checksum;

                return _expected + 6;
            }

        private:

            uint8_t _state;
            message_buffer_t _buffer;
            uint8_t _expected;
            uint8_t _received;
            uint8_t _checksum;
            uint8_t _index;
            uint8_t _id;
    };

}
