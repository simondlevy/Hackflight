/*
   mspparser.hpp: header-only implementation of MSP parsing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2018

   This program is part of Hackflight

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as 
   published by the Free Software Foundation, either version 3 of the 
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,     
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License 
   along with this code.  If not, see <http:#www.gnu.org/licenses/>.
 */


#pragma once

#include <stdint.h>
#include <string.h>

namespace hf {

    class MspParser {

        private:

            static const int INBUF_SIZE  = 128;
            static const int OUTBUF_SIZE = 128;

            typedef enum serialState_t {
                IDLE,
                HEADER_START,
                HEADER_M,
                HEADER_ARROW,
                HEADER_SIZE,
                HEADER_CMD
            } serialState_t;

            uint8_t _checksum;
            uint8_t _inBuf[INBUF_SIZE];
            uint8_t _inBufIndex;
            uint8_t _outBuf[OUTBUF_SIZE];
            uint8_t _outBufIndex;
            uint8_t _outBufSize;
            uint8_t _command;
            uint8_t _offset;
            uint8_t _dataSize;
            uint8_t _direction;

            serialState_t  _state;

            void serialize8(uint8_t a)
            {
                _outBuf[_outBufSize++] = a;
                _checksum ^= a;
            }

            void serialize16(int16_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
            }

            uint8_t read8(void)
            {
                return _inBuf[_inBufIndex++] & 0xff;
            }

            uint16_t read16(void)
            {
                uint16_t t = read8();
                t += (uint16_t)read8() << 8;
                return t;
            }

            uint32_t read32(void)
            {
                uint32_t t = read16();
                t += (uint32_t)read16() << 16;
                return t;
            }

            void serialize32(uint32_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
                serialize8((a >> 16) & 0xFF);
                serialize8((a >> 24) & 0xFF);
            }

            void headSerialResponse(uint8_t err, uint8_t s)
            {
                serialize8('$');
                serialize8('M');
                serialize8(err ? '!' : '>');
                _checksum = 0;               // start calculating a new _checksum
                serialize8(s);
                serialize8(_command);
            }

            void headSerialReply(uint8_t s)
            {
                headSerialResponse(0, s);
            }

            void prepareToSendFloats(uint8_t count)
            {
                _outBufSize = 0;
                _outBufIndex = 0;
                headSerialReply(count*4);
            }

            void sendFloat(float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            static uint8_t CRC8(uint8_t * data, int n) 
            {
                uint8_t crc = 0x00;

                for (int k=0; k<n; ++k) {

                    crc ^= data[k];
                }

                return crc;
            }


        protected:

            void init(void)
            {
                _checksum = 0;
                _outBufIndex = 0;
                _outBufSize = 0;
                _command = 0;
                _offset = 0;
                _dataSize = 0;
                _state = IDLE;
            }

            // returns true if reboot request, false otherwise
            bool update(uint8_t c)
            {
                switch (_state) {

                    case IDLE:
                        if (c == 'R') {
                            return true; // got reboot command
                        }
                        _state = (c == '$') ? HEADER_START : IDLE;
                        break;

                    case HEADER_START:
                        _state = (c == 'M') ? HEADER_M : IDLE;
                        break;

                    case HEADER_M:
                        switch (c) {
                           case '>':
                                _direction = 1;
                                _state = HEADER_ARROW;
                                break;
                            case '<':
                                _direction = 0;
                                _state = HEADER_ARROW;
                                break;
                             default:
                                _state = IDLE;
                        }
                        break;

                    case HEADER_ARROW:
                        if (c > INBUF_SIZE) {       // now we are expecting the payload size
                            _state = IDLE;
                            return false;
                        }
                        _dataSize = c;
                        _offset = 0;
                        _checksum = 0;
                        _inBufIndex = 0;
                        _checksum ^= c;
                        _state = HEADER_SIZE;      // the command is to follow
                        break;

                    case HEADER_SIZE:
                        _command = c;
                        _checksum ^= c;
                        _state = HEADER_CMD;
                        break;

                    case HEADER_CMD:
                        if (_offset < _dataSize) {
                            _checksum ^= c;
                            _inBuf[_offset++] = c;
                        } else  {
                            if (_checksum == c) {        // compare calculated and transferred _checksum
                                dispatchMessage();
                                serialize8(_checksum);                            
                            }
                            _state = IDLE;
                        }

                } // switch (_state)

                return false; // no reboot 

            } // update

            uint8_t availableBytes(void)
            {
                return _outBufSize;
            }

            uint8_t readByte(void)
            {
                _outBufSize--;
                return _outBuf[_outBufIndex++];
            }


