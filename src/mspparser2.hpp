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

class MspMessage {

    friend class MspParser;

    protected:

    static const int MAXBUF = 256;

    uint8_t _bytes[MAXBUF];
    int _pos;
    int _len;

    public:

    uint8_t start(void) 
    {
        _pos = 0;
        return getNext();
    }

    bool hasNext(void) 
    {
        return _pos <= _len;
    }


    uint8_t getNext(void) 
    {
        return _bytes[_pos++];
    }

};

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

    serialState_t  _state;

    static uint8_t CRC8(uint8_t * data, int n) 
    {
        uint8_t crc = 0x00;

        for (int k=0; k<n; ++k) {

            crc ^= data[k];
        }

        return crc;
    }

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
                _state = (c == '<') ? HEADER_ARROW : IDLE;
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


    void dispatchMessage(void)
    {
        switch (_command) {

            case 123: {

                float estalt = 0;
                float vario = 0;
                handle_GET_ALTITUDE_METERS(estalt, vario);
                } break;

            case 216: {

                uint8_t flag = 0;
                memcpy(&flag,  &_inBuf[0], sizeof(uint8_t));

                handle_SET_ARMED(flag);
                } break;

            case 121: {

                float c1 = 0;
                float c2 = 0;
                float c3 = 0;
                float c4 = 0;
                float c5 = 0;
                float c6 = 0;
                handle_GET_RC_NORMAL(c1, c2, c3, c4, c5, c6);
                } break;

            case 122: {

                float roll = 0;
                float pitch = 0;
                float yaw = 0;
                handle_GET_ATTITUDE_RADIANS(roll, pitch, yaw);
                } break;

            case 215: {

                float m1 = 0;
                memcpy(&m1,  &_inBuf[0], sizeof(float));

                float m2 = 0;
                memcpy(&m2,  &_inBuf[4], sizeof(float));

                float m3 = 0;
                memcpy(&m3,  &_inBuf[8], sizeof(float));

                float m4 = 0;
                memcpy(&m4,  &_inBuf[12], sizeof(float));

                handle_SET_MOTOR_NORMAL(m1, m2, m3, m4);
                } break;

            case 126: {

                uint8_t agl = 0;
                uint8_t flowx = 0;
                uint8_t flowy = 0;
                handle_GET_LOITER_RAW(agl, flowx, flowy);
                } break;

        }
    }

    protected:

        virtual void handle_GET_ALTITUDE_METERS(float & estalt, float & vario)
        {
            (void)estalt;
            (void)vario;
        }

        virtual void handle_SET_ARMED(uint8_t  flag)
        {
            (void)flag;
        }

        virtual void handle_GET_RC_NORMAL(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
        {
            (void)c1;
            (void)c2;
            (void)c3;
            (void)c4;
            (void)c5;
            (void)c6;
        }

        virtual void handle_GET_ATTITUDE_RADIANS(float & roll, float & pitch, float & yaw)
        {
            (void)roll;
            (void)pitch;
            (void)yaw;
        }

        virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
        {
            (void)m1;
            (void)m2;
            (void)m3;
            (void)m4;
        }

        virtual void handle_GET_LOITER_RAW(uint8_t & agl, uint8_t & flowx, uint8_t & flowy)
        {
            (void)agl;
            (void)flowx;
            (void)flowy;
        }

    public:

        MspMessage serialize_GET_ALTITUDE_METERS_Request()
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 60;
            msg._bytes[3] = 0;
            msg._bytes[4] = 123;
            msg._bytes[5] = 123;

            msg._len = 6;

            return msg;
        }

        MspMessage serialize_GET_ALTITUDE_METERS(float  estalt, float  vario)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 8;
            msg._bytes[4] = 123;

            memcpy(&msg._bytes[5], &estalt, sizeof(float));
            memcpy(&msg._bytes[9], &vario, sizeof(float));

            msg._bytes[13] = CRC8(&msg._bytes[3], 10);

            msg._len = 14;

            return msg;
        }

        MspMessage serialize_SET_ARMED(uint8_t  flag)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 1;
            msg._bytes[4] = 216;

            memcpy(&msg._bytes[5], &flag, sizeof(uint8_t));

            msg._bytes[6] = CRC8(&msg._bytes[3], 3);

            msg._len = 7;

            return msg;
        }

        MspMessage serialize_GET_RC_NORMAL_Request()
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 60;
            msg._bytes[3] = 0;
            msg._bytes[4] = 121;
            msg._bytes[5] = 121;

            msg._len = 6;

            return msg;
        }

        MspMessage serialize_GET_RC_NORMAL(float  c1, float  c2, float  c3, float  c4, float  c5, float  c6)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 24;
            msg._bytes[4] = 121;

            memcpy(&msg._bytes[5], &c1, sizeof(float));
            memcpy(&msg._bytes[9], &c2, sizeof(float));
            memcpy(&msg._bytes[13], &c3, sizeof(float));
            memcpy(&msg._bytes[17], &c4, sizeof(float));
            memcpy(&msg._bytes[21], &c5, sizeof(float));
            memcpy(&msg._bytes[25], &c6, sizeof(float));

            msg._bytes[29] = CRC8(&msg._bytes[3], 26);

            msg._len = 30;

            return msg;
        }

        MspMessage serialize_GET_ATTITUDE_RADIANS_Request()
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 60;
            msg._bytes[3] = 0;
            msg._bytes[4] = 122;
            msg._bytes[5] = 122;

            msg._len = 6;

            return msg;
        }

        MspMessage serialize_GET_ATTITUDE_RADIANS(float  roll, float  pitch, float  yaw)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 12;
            msg._bytes[4] = 122;

            memcpy(&msg._bytes[5], &roll, sizeof(float));
            memcpy(&msg._bytes[9], &pitch, sizeof(float));
            memcpy(&msg._bytes[13], &yaw, sizeof(float));

            msg._bytes[17] = CRC8(&msg._bytes[3], 14);

            msg._len = 18;

            return msg;
        }

        MspMessage serialize_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 16;
            msg._bytes[4] = 215;

            memcpy(&msg._bytes[5], &m1, sizeof(float));
            memcpy(&msg._bytes[9], &m2, sizeof(float));
            memcpy(&msg._bytes[13], &m3, sizeof(float));
            memcpy(&msg._bytes[17], &m4, sizeof(float));

            msg._bytes[21] = CRC8(&msg._bytes[3], 18);

            msg._len = 22;

            return msg;
        }

        MspMessage serialize_GET_LOITER_RAW_Request()
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 60;
            msg._bytes[3] = 0;
            msg._bytes[4] = 126;
            msg._bytes[5] = 126;

            msg._len = 6;

            return msg;
        }

        MspMessage serialize_GET_LOITER_RAW(uint8_t  agl, uint8_t  flowx, uint8_t  flowy)
        {
            MspMessage msg;

            msg._bytes[0] = 36;
            msg._bytes[1] = 77;
            msg._bytes[2] = 62;
            msg._bytes[3] = 3;
            msg._bytes[4] = 126;

            memcpy(&msg._bytes[5], &agl, sizeof(uint8_t));
            memcpy(&msg._bytes[6], &flowx, sizeof(uint8_t));
            memcpy(&msg._bytes[7], &flowy, sizeof(uint8_t));

            msg._bytes[8] = CRC8(&msg._bytes[3], 5);

            msg._len = 9;

            return msg;
        }

};

