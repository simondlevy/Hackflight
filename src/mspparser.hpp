/*
   mspparser.hpp : parser for MSP (Multiwii Serial Protocol) messages

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

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
        uint8_t _cmdMsp;
        uint8_t _offset;
        uint8_t _dataSize;

        serialState_t  _parserState;

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
            serialize8(_cmdMsp);
        }

        void headSerialReply(uint8_t s)
        {
            headSerialResponse(0, s);
        }

        protected:

        void sendFloats(float * src, uint8_t count)
        {
            _outBufSize = 0;
            _outBufIndex = 0;

            headSerialReply(4*count);

            for (uint8_t k=0; k<count; ++k) {
                uint32_t a;
                memcpy(&a, &src[k], 4);
                serialize32(a);
            }
        }

        void receiveFloats(float * dst, uint8_t count)
        {
            for (uint8_t k=0; k<count; ++k) {
                dst[k] = readFloat();
            }
            headSerialReply(0);
        }

        bool readBool(void)
        {
            bool retval = (bool)read8();
            headSerialReply(0);
            return retval;
        }

        float readFloat(void)
        {
            float f = 0;
            uint32_t t = read32();
            memcpy(&f, &t, 4);
            return f;
        }

        void error(void)
        {
            headSerialResponse(1, 0);
        }

        // See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
        static const uint8_t CMD_GET_RC_NORMAL        = 121;
        static const uint8_t CMD_GET_ATTITUDE_RADIANS = 122; 
        static const uint8_t CMD_GET_ALTITUDE_METERS  = 123; 
        static const uint8_t CMD_SET_MOTOR_NORMAL     = 215;    
        static const uint8_t CMD_SET_ARMED            = 216;    

        void init(void)
        {
            _checksum = 0;
            _outBufIndex = 0;
            _outBufSize = 0;
            _cmdMsp = 0;
            _offset = 0;
            _dataSize = 0;
            _parserState = IDLE;
        }

        // returns true if reboot request, false otherwise
        bool update(uint8_t c)
        {
            switch (_parserState) {

                case IDLE:
                    if (c == 'R') {
                        return true; // got reboot command
                    }
                    _parserState = (c == '$') ? HEADER_START : IDLE;
                    break;
                case HEADER_START:
                    _parserState = (c == 'M') ? HEADER_M : IDLE;
                    break;
                case HEADER_M:
                    _parserState = (c == '<') ? HEADER_ARROW : IDLE;
                    break;
                case HEADER_ARROW:
                    if (c > INBUF_SIZE) {       // now we are expecting the payload size
                        _parserState = IDLE;
                        return false;
                    }
                    _dataSize = c;
                    _offset = 0;
                    _checksum = 0;
                    _inBufIndex = 0;
                    _checksum ^= c;
                    _parserState = HEADER_SIZE;      // the command is to follow
                    break;
                case HEADER_SIZE:
                    _cmdMsp = c;
                    _checksum ^= c;
                    _parserState = HEADER_CMD;
                    break;
                case HEADER_CMD:
                    if (_offset < _dataSize) {
                        _checksum ^= c;
                        _inBuf[_offset++] = c;
                    } else  {
                        if (_checksum == c) {        // compare calculated and transferred _checksum
                            dispatchCommand(_cmdMsp);
                            serialize8(_checksum);                            
                        }
                        _parserState = IDLE;
                    }

            } // switch (_parserState)

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

        virtual void dispatchCommand(uint8_t cmd) = 0;

    }; // class MspParser


} // namespace
