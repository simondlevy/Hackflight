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

namespace hf {

    class MspParser {

        public:

            /**
             * Returns message ID or 0 for not  ready
             */
            uint8_t parse(const uint8_t byte)
            {
                uint8_t result = 0;

                switch (_state) {

                    case 0:
                        if (byte == 36) {  // $
                            _state++;
                        }
                        break;

                    case 1:
                        if (byte == 77) { // M
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
                        _message_length_expected = byte;
                        _message_checksum = byte;
                        _message_index = 0;
                        _state++;
                        break;

                    case 4:
                        _message_id = byte;
                        _message_length_received = 0;
                        _message_checksum ^= byte;
                        if (_message_length_expected > 0) {
                            // process payload
                            _state++;
                        }
                        else {
                            // no payload
                            _state += 2;
                        }
                        break;

                    case 5:
                        _message_buffer[_message_index++] = byte;
                        _message_checksum ^= byte;
                        _message_length_received++;
                        if (_message_length_received >= _message_length_expected) {
                            _state++;
                        }
                        break;

                    case 6:
                        if (_message_checksum == byte) {
                            result = _message_id;
                        }
                        // Reset variables
                        _message_length_received = 0;
                        _state = 0;
                        break;
                }

                return result;
            }

            float getFloat(const uint8_t index)
            {
                const uint8_t offset = 4 * index;
                uint32_t tmp = (uint32_t) (
                        _message_buffer[offset+3] << 24 |
                        _message_buffer[offset+2] << 16 |
                        _message_buffer[offset+1] << 8 |
                        _message_buffer[offset]);
                float value = 0;
                memcpy(&value, &tmp, 4);
                return value;
            }

            uint16_t getUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (_message_buffer[offset+1] << 8) | _message_buffer[offset];
                return value;
            }

            uint8_t getByte(const uint8_t index)
            {
                return _message_buffer[index];
            }

        private:

            uint8_t _state;
            uint8_t _message_buffer[256];
            uint8_t _message_length_expected;
            uint8_t _message_length_received;
            uint8_t _message_checksum;
            uint8_t _message_index;
            uint8_t _message_id;

    };

}
