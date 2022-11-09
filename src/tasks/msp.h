/*
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

#include "core/vstate.h"
#include "esc.h"
#include "imu.h"
#include "maths.h"
#include "receiver.h"
#include "task.h"

class Msp : public Task {

    friend class Hackflight;

    static const uint8_t MAXMSG = 255;

    static const int OUTBUF_SIZE = 128;

    Board *         m_board;
    Esc *           m_esc;
    Arming *        m_arming;
    Receiver *      m_receiver;
    VehicleState *  m_vstate;

    uint8_t _outBufChecksum;
    uint8_t _outBuf[OUTBUF_SIZE];
    uint8_t _outBufIndex;
    uint8_t _outBufSize;

    uint8_t _payload[128] = {};

    float  motors[MAX_SUPPORTED_MOTORS];

    bool    _gotRebootRequest;

    void serialize16(int16_t a)
    {
        serialize8(a & 0xFF);
        serialize8((a >> 8) & 0xFF);
    }

    void prepareToSend(uint8_t type, uint8_t count, uint8_t size)
    {
        _outBufSize = 0;
        _outBufIndex = 0;
        _outBufChecksum = 0;

        addToOutBuf('$');
        addToOutBuf('M');
        addToOutBuf('>');
        serialize8(count*size);
        serialize8(type);
    }

    void addToOutBuf(uint8_t a)
    {
        _outBuf[_outBufSize++] = a;
    }

    void completeSend(void)
    {
        serialize8(_outBufChecksum);
        Serial.write(_outBuf, _outBufSize);
    }

    void serialize8(uint8_t a)
    {
        addToOutBuf(a);
        _outBufChecksum ^= a;
    }

    void prepareToSendBytes(uint8_t type, uint8_t count)
    {
        prepareToSend(type, count, 1);
    }

    void sendByte(uint8_t src)
    {
        serialize8(src);
    }

    void prepareToSendShorts(uint8_t type, uint8_t count)
    {
        prepareToSend(type, count, 2);
    }

    void sendShort(uint16_t src)
    {
        uint16_t a;
        memcpy(&a, &src, 2);
        serialize16(a);
    }

    void prepareToSendInts(uint8_t type, uint8_t count)
    {
        prepareToSend(type, count, 4);
    }

    void prepareToSendFloats(uint8_t type, uint8_t count)
    {
        prepareToSend(type, count, 4);
    }

    uint8_t availableBytes(void)
    {
        return _outBufSize;
    }

    uint8_t readByte(void)
    {
        _outBufSize--;
        return _outBuf[_outBufIndex++];
    }

    void parse(uint8_t c)
    {
        enum {
            IDLE,
            GOT_START,
            GOT_M,
            GOT_ARROW,
            GOT_SIZE,
            IN_PAYLOAD,
            GOT_CRC
        }; 

        static uint8_t parser_state;

        static uint8_t type;
        static uint8_t crc;
        static uint8_t size;
        static uint8_t index;

        if (parser_state == IDLE && c == 'R') {
            _gotRebootRequest = true;
            m_board->reboot();
        }

        // Payload functions
        size = parser_state == GOT_ARROW ? c : size;
        index = parser_state == IN_PAYLOAD ? index + 1 : 0;
        bool incoming = type >= 200;
        bool in_payload = incoming && parser_state == IN_PAYLOAD;

        // Command acquisition function
        type = parser_state == GOT_SIZE ? c : type;

        // Parser state transition function (final transition below)
        parser_state
            = parser_state == IDLE && c == '$' ? GOT_START
            : parser_state == GOT_START && c == 'M' ? GOT_M
            : parser_state == GOT_M && (c == '<' || c == '>') ? GOT_ARROW
            : parser_state == GOT_ARROW ? GOT_SIZE
            : parser_state == GOT_SIZE ? IN_PAYLOAD
            : parser_state == IN_PAYLOAD && index <= size ? IN_PAYLOAD
            : parser_state == IN_PAYLOAD ? GOT_CRC
            : parser_state;

        // Checksum transition function
        crc 
            = parser_state == GOT_SIZE ?  c
            : parser_state == IN_PAYLOAD ? crc ^ c
            : parser_state == GOT_CRC ? crc 
            : 0;

        // Payload accumulation
        if (in_payload) {
            _payload[index-1] = c;
        }

        if (parser_state == GOT_CRC) {

            // Message dispatch
            if (crc == c) {
                dispatchMessage(type);
            }

            parser_state = IDLE;
        }


    } // parse

    static float scale(const float value)
    {
        return 1000 + 1000 * value;
    }

    static int16_t rad2degi(float rad)
    {
        return (int16_t)Math::rad2deg(rad);
    }

    void handle_SET_MOTOR(uint16_t  m1, uint16_t  m2, uint16_t  m3, uint16_t  m4)
    {
        motors[0] = m_esc->convertFromExternal(m1);
        motors[1] = m_esc->convertFromExternal(m2);
        motors[2] = m_esc->convertFromExternal(m3);
        motors[3] = m_esc->convertFromExternal(m4);
    }

    void handle_RC_Request(
            uint16_t & c1,
            uint16_t & c2,
            uint16_t & c3,
            uint16_t & c4,
            uint16_t & c5,
            uint16_t & c6)
    {
        c1 = (uint16_t)m_receiver->getRawThrottle();
        c2 = (uint16_t)m_receiver->getRawRoll();
        c3 = (uint16_t)m_receiver->getRawPitch();
        c4 = (uint16_t)m_receiver->getRawYaw();
        c5 = scale(m_receiver->getRawAux1());
        c6 = scale(m_receiver->getRawAux2());
     }

    void handle_ATTITUDE_Request(uint16_t & phi, uint16_t & theta, uint16_t & psi)
    {
        phi   = 10 * rad2degi(m_vstate->phi);
        theta = 10 * rad2degi(m_vstate->theta);
        psi   = rad2degi(m_vstate->psi);
    }

    void dispatchMessage(uint8_t command)
    {
        switch (command) {

            case 214:
                {
                    uint16_t m1 = 0;
                    memcpy(&m1,  &_payload[0], sizeof(uint16_t));

                    uint16_t m2 = 0;
                    memcpy(&m2,  &_payload[2], sizeof(uint16_t));

                    uint16_t m3 = 0;
                    memcpy(&m3,  &_payload[4], sizeof(uint16_t));

                    uint16_t m4 = 0;
                    memcpy(&m4,  &_payload[6], sizeof(uint16_t));

                    handle_SET_MOTOR(m1, m2, m3, m4);

                } break;

            case 105:
                {
                    uint16_t c1 = 0;
                    uint16_t c2 = 0;
                    uint16_t c3 = 0;
                    uint16_t c4 = 0;
                    uint16_t c5 = 0;
                    uint16_t c6 = 0;
                    handle_RC_Request(c1, c2, c3, c4, c5, c6);
                    prepareToSendShorts(command, 6);
                    sendShort(c1);
                    sendShort(c2);
                    sendShort(c3);
                    sendShort(c4);
                    sendShort(c5);
                    sendShort(c6);
                    completeSend();
                } break;

            case 108:
                {
                    uint16_t phi = 0;
                    uint16_t theta = 0;
                    uint16_t psi = 0;
                    handle_ATTITUDE_Request(phi, theta, psi);
                    prepareToSendShorts(command, 3);
                    sendShort(phi);
                    sendShort(theta);
                    sendShort(psi);
                    completeSend();
                } break;

        } // switch (_command)

    } // dispatchMessage 

    Msp() : Task(100) { } // Hz

    void begin(
            Board * board,
            Esc * esc,
            Arming * arming,
            Receiver * receiver,
            VehicleState * vstate)
    {
        m_board = board;

        m_esc = esc;

        m_arming = arming;

        Serial.begin(115200);

        m_vstate = vstate;

        m_receiver = receiver;

        _outBufChecksum = 0;
        _outBufIndex = 0;
        _outBufSize = 0;
    }

    virtual void fun(uint32_t usec) override
    {
        (void)usec;

        while (Serial.available()) {
            parse(Serial.read());
        }
    }

}; // class Msp
