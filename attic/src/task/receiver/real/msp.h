/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along
   with Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "task/receiver/real.h"
#include "msp/parser.h"

class MspReceiver : public RealReceiver {

    private:

        static void dump(int16_t val)
        {
            Serial.print(val);
            Serial.print("  ");
        }

        MspParser m_parser;

        float m_throttle;
        float m_roll;
        float m_pitch;
        float m_yaw;
        float m_aux1;
        float m_aux2;

        uint32_t m_frameTimeUs;

        volatile bool m_done;

    protected:

        virtual bool devRead(
                float & throttle,
                float & roll,
                float & pitch,
                float & yaw,
                float & aux1,
                float & aux2,
                uint32_t & frameTimeUs) override
        {
            bool done = m_done;

            if (done) {
                throttle = m_throttle;
                roll     = m_roll;
                pitch    = m_pitch;
                yaw      = m_yaw;
                aux1     = m_aux1;
                aux2     = m_aux2;

                frameTimeUs = m_frameTimeUs;

                m_done = false;
            }

            return done;
        }

        virtual void parse(const uint8_t c) override
        {
            if (m_parser.parse(c) == 200) {

                m_throttle = (float)m_parser.parseShort(0);
                m_roll     = (float)m_parser.parseShort(1);
                m_pitch    = (float)m_parser.parseShort(2);
                m_yaw      = (float)m_parser.parseShort(3);
                m_aux1     = ((float)m_parser.parseShort(4) - 1000) / 1000;
                m_aux2     = ((float)m_parser.parseShort(5) - 1000) / 1000;

                m_frameTimeUs = micros();

                m_done = true;

                /*
                   dump(c1);
                   dump(c2);
                   dump(c3);
                   dump(c4);
                   dump(c5);
                   dump(c6);
                   Serial.println();
                 */
            }
        }

}; // class MspReceiver


