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

#include "task.h"
#include "esc.h"
#include "imu.h"
#include "msp/parser.h"
#include "msp/serializer.h"
#include "receiver.h"

class UsbTask : public Task {

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

        MspParser m_parser;
        MspSerializer m_serializer;

        bool m_gotRebootRequest;

        void sendOutBuf(void)
        {
            Serial.write(m_serializer.outBuf, m_serializer.outBufSize);
        }

    protected:

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

            while (Serial.available()) {

                uint8_t byte = Serial.read();

                //Serial4.println(byte, HEX);

                if (m_parser.isIdle() && byte == 'R') {
                    m_gotRebootRequest = true;
                }

                uint8_t messageType = m_parser.parse(byte);

                switch (messageType) {

                    case 105: // RC
                        {
                            uint16_t c1 = (uint16_t)m_receiver->getRawThrottle();
                            uint16_t c2 = (uint16_t)m_receiver->getRawRoll();
                            uint16_t c3 = (uint16_t)m_receiver->getRawPitch();
                            uint16_t c4 = (uint16_t)m_receiver->getRawYaw();
                            uint16_t c5 = scale(m_receiver->getRawAux1());
                            uint16_t c6 = scale(m_receiver->getRawAux2());
                            m_serializer.serializeRawRc(105, c1, c2, c3, c4, c5, c6);
                            sendOutBuf();

                        } break;

                    case 108: // ATTITUDE
                        {
                            m_serializer.prepareToSerializeShorts(messageType, 3);
                            m_serializer.serializeShort(10 * rad2degi(m_vstate->phi));
                            m_serializer.serializeShort(10 * rad2degi(m_vstate->theta));
                            m_serializer.serializeShort(rad2degi(m_vstate->psi));
                            m_serializer.completeSerialize();
                            sendOutBuf();
                        } break;

                    case 214: // SET_MOTORS
                        {
                            motors[0] =
                                m_esc->convertFromExternal(m_parser.parseShort(0));
                            motors[1] =
                                m_esc->convertFromExternal(m_parser.parseShort(1));
                            motors[2] =
                                m_esc->convertFromExternal(m_parser.parseShort(2));
                            motors[3] =
                                m_esc->convertFromExternal(m_parser.parseShort(3));

                        } break;

                    default:
                        break;
                }
            }
        }

    public:

        UsbTask(void) 
            : Task(100) // Hz
        { 
        }

        float motors[MAX_SUPPORTED_MOTORS];

        void begin(
                Esc * esc,
                Arming * arming,
                Receiver * receiver,
                VehicleState * vstate)
        {
            Serial.begin(115200);

            m_esc = esc;
            m_arming = arming;
            m_vstate = vstate;
            m_receiver = receiver;
        }

        bool gotRebootRequest(void)
        {
            return m_gotRebootRequest;
        }

}; // class UsbMsp
