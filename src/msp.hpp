/*
   Multiwii Serial Protocol support for Hackflight

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

    class Msp {

        private:

            static const uint8_t BUF_SIZE = 128;

            uint8_t _payloadChecksum;
            uint8_t _payloadIndex;

            uint8_t _state;
            uint8_t _message_buffer[256];
            uint8_t _message_length_expected;
            uint8_t _message_length_received;
            uint8_t _message_checksum;
            uint8_t _message_index;
            uint8_t _message_id;

            void serialize32(const int32_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
                serialize8((a >> 16) & 0xFF);
                serialize8((a >> 24) & 0xFF);
            }

            void serialize16(const int16_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
            }

            void serialize8(const uint8_t a)
            {
                addToOutBuf(a);
                _payloadChecksum ^= a;
            }

            void prepareToSerialize(
                    const uint8_t type, const uint8_t count, const uint8_t size)
            {
                payloadSize = 0;
                _payloadIndex = 0;
                _payloadChecksum = 0;

                addToOutBuf('$');
                addToOutBuf('M');
                addToOutBuf('>');
                serialize8(count*size);
                serialize8(type);
            }

            void addToOutBuf(const uint8_t a)
            {
                payload[payloadSize++] = a;
            }

            void prepareToSerializeBytes(const uint8_t type, const uint8_t count)
            {
                prepareToSerialize(type, count, 1);
            }

            void serializeByte(const uint8_t src)
            {
                serialize8(src);
            }

            void prepareToSerializeInts(const uint8_t msgtype, const uint8_t count)
            {
                prepareToSerialize(msgtype, count, 4);
            }

            void prepareToSerializeFloats(const uint8_t msgtype, const uint8_t count)
            {
                prepareToSerialize(msgtype, count, 4);
            }

            void prepareToSerializeShorts(const uint8_t msgtype, const uint8_t count)
            {
                prepareToSerialize(msgtype, count, 2);
            }

            void completeSerialize(void)
            {
                serialize8(_payloadChecksum);
                _payloadIndex = 0;
            }

            void serializeFloat(const float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void serializeShort(const uint16_t src)
            {
                uint16_t a;
                memcpy(&a, &src, 2);
                serialize16(a);
            }

        public:
    
            uint8_t payload[BUF_SIZE];

            uint8_t payloadSize;

            /**
             * Returns message ID or 0 for not  ready
             */
             uint8_t parse(const uint8_t byte)
            {
                uint8_t result = 0;

                if (_state == 0) { // sync char 1
                    if (byte == 36) {  // $
                        _state++;
                    }
                }

                else if (_state == 1)  { // sync char 2
                    if (byte == 77) { // M
                        _state++;
                    }
                    else {  // restart and try again
                        _state = 0;
                    }
                }

                else if (_state == 2)  { // direction
                    _state++;
                }

                else if (_state == 3) {
                    _message_length_expected = byte;
                    _message_checksum = byte;
                    _message_index = 0;
                    _state++;
                }

                else if (_state == 4) {
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
                 }

                else if (_state == 5)  { // payload
                    _message_buffer[_message_index++] = byte;
                    _message_checksum ^= byte;
                    _message_length_received++;
                    if (_message_length_received >= _message_length_expected) {
                        _state++;
                    }
                }

                else if (_state == 6) {
                    if (_message_checksum == byte) {
                        result = _message_id;
                    }
                    // Reset variables
                    _message_length_received = 0;
                    _state = 0;
                }

                return result;
            }

            uint16_t getUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (_message_buffer[offset+1] << 8) | _message_buffer[offset];
                return value;
            }

            void serializeFloats(
                    const uint8_t messageType, const float src[], const uint8_t count)
            {
                prepareToSerializeFloats(messageType, count);

                for (auto k=0; k<count; ++k) {
                    serializeFloat(src[k]);
                }

                completeSerialize();
            }

            void serializeShorts(
                    const uint8_t messageType, const int16_t src[], const uint8_t count)
            {
                prepareToSerializeShorts(messageType, count);

                for (auto k=0; k<count; ++k) {
                    serializeShort(src[k]);
                }

                completeSerialize();
            }

            uint8_t available(void)
            {
                return payloadSize;
            }

            uint8_t read(void)
            {
                payloadSize--;
                return payload[_payloadIndex++];
            }

    };

}
