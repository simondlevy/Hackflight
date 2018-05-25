/*
   msp.hpp : MSP (Multiwii Serial Protocol) support

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

#include "receiver.hpp"
#include "mixer.hpp"
#include "datatypes.hpp"

namespace hf {

    class MSP {

        friend class Hackflight;

        private:

        // See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
        static const uint8_t MSP_RC_NORMAL        =    121;
        static const uint8_t MSP_ATTITUDE_RADIANS =    122; 
        static const uint8_t MSP_SET_MOTOR_NORMAL =    215;    
        static const uint8_t MSP_SET_ARMED        =    216;    

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

        vehicle_state_t * _vehicle_state;
        Receiver * _receiver;
        Mixer * _mixer;

        uint8_t         checksum;
        uint8_t         inBuf[INBUF_SIZE];
        uint8_t         inBufIndex;
        uint8_t         outBuf[OUTBUF_SIZE];
        uint8_t         outBufIndex;
        uint8_t         outBufSize;
        uint8_t         cmdMSP;
        uint8_t         offset;
        uint8_t         dataSize;
        serialState_t   _parser_state;

        void serialize8(uint8_t a)
        {
            outBuf[outBufSize++] = a;
            checksum ^= a;
        }

        void serialize16(int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        uint8_t read8(void)
        {
            return inBuf[inBufIndex++] & 0xff;
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

        float readFloat(void)
        {
            float f = 0;
            uint32_t t = read32();
            memcpy(&f, &t, 4);
            return f;
        }

        void serializeFloats(float f[], uint8_t n)
        {
            headSerialReply(4*n);

            for (uint8_t k=0; k<n; ++k) {
                uint32_t a;
                memcpy(&a, &f[k], 4);
                serialize32(a);
            }
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
            checksum = 0;               // start calculating a new checksum
            serialize8(s);
            serialize8(cmdMSP);
        }

        void headSerialReply(uint8_t s)
        {
            headSerialResponse(0, s);
        }

        void headSerialError(uint8_t s)
        {
            headSerialResponse(1, s);
        }

        void tailSerialReply(void)
        {
            serialize8(checksum);
        }

        void dispatchCommand(void)
        {
            switch (cmdMSP) {

                case MSP_SET_MOTOR_NORMAL:
                    for (uint8_t i = 0; i < _mixer->nmotors; i++)
                        _mixer->motorsDisarmed[i] = readFloat();
                    headSerialReply(0);
                    break;

                case MSP_SET_ARMED:
                    if (read8()) {  // got arming command: arm only if throttle is down
                        if (_receiver->throttleIsDown()) {
                            _vehicle_state->armed = true;
                        }
                    }
                    else {          // got disarming command: always disarm
                        _vehicle_state->armed = false;
                    }
                    headSerialReply(0);
                    break;

                case MSP_RC_NORMAL:
                    outBufSize = 0;
                    outBufIndex = 0;
                    serializeFloats(_receiver->rawvals, 8);
                    break;

                case MSP_ATTITUDE_RADIANS: 
                    {
                        outBufSize = 0;
                        outBufIndex = 0;
                        serializeFloats(_vehicle_state->eulerAngles, 3);
                    }
                    break;

                    // don't know how to handle the (valid) message, indicate error
                default:                   
                    headSerialError(0);
                    break;
            }
        }

        protected:

        void init(vehicle_state_t * vehicle_state, Receiver * receiver, Mixer * mixer)
        {
            _vehicle_state = vehicle_state;
            _receiver = receiver;
            _mixer = mixer;

            checksum = 0;
            outBufIndex = 0;
            outBufSize = 0;
            cmdMSP = 0;
            offset = 0;
            dataSize = 0;
            _parser_state = IDLE;
        }

        void update(uint8_t c)
        {
            switch (_parser_state) {
                case IDLE:
                    _parser_state = (c == '$') ? HEADER_START : IDLE;
                    break;
                case HEADER_START:
                    _parser_state = (c == 'M') ? HEADER_M : IDLE;
                    break;
                case HEADER_M:
                    _parser_state = (c == '<') ? HEADER_ARROW : IDLE;
                    break;
                case HEADER_ARROW:
                    if (c > INBUF_SIZE) {       // now we are expecting the payload size
                        _parser_state = IDLE;
                        return;
                    }
                    dataSize = c;
                    offset = 0;
                    checksum = 0;
                    inBufIndex = 0;
                    checksum ^= c;
                    _parser_state = HEADER_SIZE;      // the command is to follow
                    break;
                case HEADER_SIZE:
                    cmdMSP = c;
                    checksum ^= c;
                    _parser_state = HEADER_CMD;
                    break;
                case HEADER_CMD:
                    if (offset < dataSize) {
                        checksum ^= c;
                        inBuf[offset++] = c;
                    } else  {
                        if (checksum == c) {        // compare calculated and transferred checksum
                            dispatchCommand();
                            tailSerialReply();
                        }
                        _parser_state = IDLE;
                    }
            } // switch (_parser_state)

        } // update

        uint8_t availableBytes(void)
        {
            return outBufSize;
        }

        uint8_t readByte(void)
        {
            outBufSize--;
            return outBuf[outBufIndex++];
        }


    }; // class MSP


} // namespace
