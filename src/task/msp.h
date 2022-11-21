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

    private:

        static const uint8_t MAXMSG = 255;

        static const int OUTBUF_SIZE = 128;

        Esc *            m_esc;
        Arming *         m_arming;
        Receiver *       m_receiver;
        VehicleState *   m_vstate;

        uint8_t m_outBufChecksum;
        uint8_t m_outBuf[OUTBUF_SIZE];
        uint8_t m_outBufIndex;
        uint8_t m_outBufSize;

        uint8_t m_payload[128] = {};

        bool m_gotRebootRequest;

        void serialize16(int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        void prepareToSend(uint8_t type, uint8_t count, uint8_t size)
        {
            m_outBufSize = 0;
            m_outBufIndex = 0;
            m_outBufChecksum = 0;

            addToOutBuf('$');
            addToOutBuf('M');
            addToOutBuf('>');
            serialize8(count*size);
            serialize8(type);
        }

        void addToOutBuf(uint8_t a)
        {
            m_outBuf[m_outBufSize++] = a;
        }

        void completeSend(void)
        {
            serialize8(m_outBufChecksum);
            serialWrite(m_outBuf, m_outBufSize);
        }

        void serialize8(uint8_t a)
        {
            addToOutBuf(a);
            m_outBufChecksum ^= a;
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
            return m_outBufSize;
        }

        uint8_t readByte(void)
        {
            m_outBufSize--;
            return m_outBuf[m_outBufIndex++];
        }

        void parse(const uint8_t c)
        {
            typedef enum {
                IDLE,
                GOT_START,
                GOT_M,
                GOT_ARROW,
                GOT_SIZE,
                IN_PAYLOAD,
                GOT_CRC
            } parser_state_t; 

            static parser_state_t _parser_state;
            static uint8_t _type;
            static uint8_t _crc;
            static uint8_t _size;
            static uint8_t _index;

            if (_parser_state == IDLE && c == 'R') {
                m_gotRebootRequest = true;
                return;
            }

            // Payload functions
            _size = _parser_state == GOT_ARROW ? c : _size;
            _index = _parser_state == IN_PAYLOAD ? _index + 1 : 0;
            const bool incoming = _type >= 200;
            const bool in_payload = incoming && _parser_state == IN_PAYLOAD;

            // Command acquisition function
            _type = _parser_state == GOT_SIZE ? c : _type;

            // Parser state transition function (final transition below)
            _parser_state
                = _parser_state == IDLE && c == '$' ? GOT_START
                : _parser_state == GOT_START && c == 'M' ? GOT_M
                : _parser_state == GOT_M && (c == '<' || c == '>') ? GOT_ARROW
                : _parser_state == GOT_ARROW ? GOT_SIZE
                : _parser_state == GOT_SIZE ? IN_PAYLOAD
                : _parser_state == IN_PAYLOAD && _index <= _size ? IN_PAYLOAD
                : _parser_state == IN_PAYLOAD ? GOT_CRC
                : _parser_state;

            // Checksum transition function
            _crc 
                = _parser_state == GOT_SIZE ?  c
                : _parser_state == IN_PAYLOAD ? _crc ^ c
                : _parser_state == GOT_CRC ? _crc 
                : 0;

            // Payload accumulation
            if (in_payload) {
                m_payload[_index-1] = c;
            }

            if (_parser_state == GOT_CRC) {

                // Message dispatch
                if (_crc == c) {
                    dispatchMessage(_type);
                }

                _parser_state = IDLE;
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
                        memcpy(&m1,  &m_payload[0], sizeof(uint16_t));

                        uint16_t m2 = 0;
                        memcpy(&m2,  &m_payload[2], sizeof(uint16_t));

                        uint16_t m3 = 0;
                        memcpy(&m3,  &m_payload[4], sizeof(uint16_t));

                        uint16_t m4 = 0;
                        memcpy(&m4,  &m_payload[6], sizeof(uint16_t));

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

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

            while (serialAvailable()) {
                parse(serialRead());
            }
        }

    protected:

        virtual void serialBegin(uint32_t baud) = 0;

        virtual uint32_t serialAvailable(void) = 0;

        virtual uint8_t serialRead(void) = 0;

        virtual void serialWrite(uint8_t buf[], uint8_t count) = 0;

    public:

        float motors[MAX_SUPPORTED_MOTORS];

        Msp() : Task(100) { } // Hz

        bool gotRebootRequest(void)
        {
            return m_gotRebootRequest;
        }

        void begin(
                Esc * esc,
                Arming * arming,
                Receiver * receiver,
                VehicleState * vstate)
        {
            serialBegin(115200);

            m_esc = esc;

            m_arming = arming;

            m_vstate = vstate;

            m_receiver = receiver;

            m_outBufChecksum = 0;
            m_outBufIndex = 0;
            m_outBufSize = 0;
        }

}; // class Msp
