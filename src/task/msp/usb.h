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

#include <stdint.h>

#include "task/msp.h"

class UsbMsp : public Msp {

    private:

        static int16_t rad2degi(float rad)
        {
            return (int16_t)Math::rad2deg(rad);
        }

        static float scale(const float value)
        {
            return 1000 + 1000 * value;
        }

        Esc *            m_esc;
        Arming *         m_arming;
        Receiver *       m_receiver;
        VehicleState *   m_vstate;

        void handleAttitudeRequest(uint16_t & phi, uint16_t & theta, uint16_t & psi)
        {
            phi   = 10 * rad2degi(m_vstate->phi);
            theta = 10 * rad2degi(m_vstate->theta);
            psi   = rad2degi(m_vstate->psi);
        }

        void handleRcRequest(
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

        void handleSetMotor(uint16_t  m1, uint16_t  m2, uint16_t  m3, uint16_t  m4)
        {
            motors[0] = m_esc->convertFromExternal(m1);
            motors[1] = m_esc->convertFromExternal(m2);
            motors[2] = m_esc->convertFromExternal(m3);
            motors[3] = m_esc->convertFromExternal(m4);
        }

    protected:

        virtual void dispatchMessage(uint8_t command) override
        {
            Msp::dispatchMessage(command);

            switch (command) {

                case 105:
                    {
                        uint16_t c1 = 0;
                        uint16_t c2 = 0;
                        uint16_t c3 = 0;
                        uint16_t c4 = 0;
                        uint16_t c5 = 0;
                        uint16_t c6 = 0;
                        handleRcRequest(c1, c2, c3, c4, c5, c6);

                        sendRawRc(105, c1, c2, c3, c4, c4, c6);

                    } break;

                case 108:
                    {
                        uint16_t phi = 0;
                        uint16_t theta = 0;
                        uint16_t psi = 0;
                        handleAttitudeRequest(phi, theta, psi);
                        prepareToSendShorts(command, 3);
                        sendShort(phi);
                        sendShort(theta);
                        sendShort(psi);
                        completeSend();
                    } break;

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

                        handleSetMotor(m1, m2, m3, m4);

                    } break;
            }
        }

        virtual void serialBegin(const uint32_t baud) override
        {
            Serial.begin(baud);
        }

        virtual uint32_t serialAvailable(void) override
        {
            return Serial.available();
        }

        virtual uint8_t serialRead(void) override
        {
            return Serial.read();
        }

        virtual void serialWrite(const uint8_t buf[], const uint8_t count) override
        {
            Serial.write(buf, count);
        }
    public:

        float motors[MAX_SUPPORTED_MOTORS];

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
        }


}; // class UsbMsp
