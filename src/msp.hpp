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

            typedef enum {
                IDLE,
                GOT_START,
                GOT_M,
                GOT_ARROW,
                GOT_SIZE,
                IN_PAYLOAD,
                GOT_CRC
            } parserState_t; 

            parserState_t _state;

            uint8_t _payloadChecksum;
            uint8_t _payloadIndex;

            uint8_t _newstate;
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
             uint8_t newparse(const uint8_t byte)
            {
                uint8_t result = 0;

                if (_newstate == 0) { // sync char 1
                    if (byte == 36) {  // $
                        _newstate++;
                    }
                }

                else if (_newstate == 1)  { // sync char 2
                    if (byte == 77) { // M
                        _newstate++;
                    }
                    else {  // restart and try again
                        _newstate = 0;
                    }
                }

                else if (_newstate == 2)  { // direction
                    _newstate++;
                }

                else if (_newstate == 3) {
                    _message_length_expected = byte;
                    _message_checksum = byte;
                    _message_index = 0;
                    _newstate++;
                }

                else if (_newstate == 4) {
                    _message_id = byte;
                    _message_length_received = 0;
                    _message_checksum ^= byte;
                    if (_message_length_expected > 0) {
                        // process payload
                        _newstate++;
                    }
                    else {
                        // no payload
                        _newstate += 2;
                    }
                 }

                else if (_newstate == 5)  { // payload
                    _message_buffer[_message_index++] = byte;
                    _message_checksum ^= byte;
                    _message_length_received++;
                    if (_message_length_received >= _message_length_expected) {
                        _newstate++;
                    }
                }

                else if (_newstate == 6) {
                    if (_message_checksum == byte) {
                        result = _message_id;
                    }
                    // Reset variables
                    _message_length_received = 0;
                    _newstate = 0;
                }

                return result;
            }

            /**
             * Returns message type or 0 for not  ready
             */
            uint8_t parse(const uint8_t c)
            {
                uint8_t messageType = 0;

                static uint8_t _type;
                static uint8_t _crc;
                static uint8_t _size;
                static uint8_t _index;

                // Payload transition functions
                _size = _state == GOT_ARROW ? c : _size;
                _index = _state == IN_PAYLOAD ? _index + 1 : 0;
                const bool isCommand = _type >= 200;
                const bool inPayload = isCommand && _state == IN_PAYLOAD;

                // Message-type transition function
                _type = _state == GOT_SIZE ? c : _type;

                // Parser state transition function (final transition below)
                _state
                    = _state == IDLE && c == '$' ? GOT_START
                    : _state == GOT_START && c == 'M' ? GOT_M
                    : _state == GOT_M && (c == '<' || c == '>') ? GOT_ARROW
                    : _state == GOT_ARROW ? GOT_SIZE
                    : _state == GOT_SIZE ? IN_PAYLOAD
                    : _state == IN_PAYLOAD && _index <= _size ? IN_PAYLOAD
                    : _state == IN_PAYLOAD ? GOT_CRC
                    : _state;

                // Checksum transition function
                _crc 
                    = _state == GOT_SIZE ?  c
                    : _state == IN_PAYLOAD ? _crc ^ c
                    : _state == GOT_CRC ? _crc 
                    : 0;

                // Payload accumulation
                if (inPayload) {
                    payload[_index-1] = c;
                }

                if (_state == GOT_CRC) {

                    // Message dispatch
                    if (_crc == c) {
                        messageType = _type;
                    }

                    _state = IDLE;
                }

                return messageType;

            } // parse

            uint16_t parseUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (_message_buffer[offset+1] << 8) | _message_buffer[offset];

                /*
                for (uint8_t k=0; k<_message_length_expected; ++k) {
                    printf("%02X ", _message_buffer[k]);
                }
                printf("\n");*/

                return value;
            }

            static uint16_t parseUshort(const uint8_t msg[256], const uint8_t index)
            {
                const uint8_t offset = 2 * index + 5;

                uint16_t value = (msg[offset+1] << 8) | msg[offset];

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
