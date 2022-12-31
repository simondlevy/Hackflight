/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "task.h"
#include "msp.h"

uint32_t g_rangerCount;
uint32_t g_mocapCount;

class SkyclientTask : public Task {

    private:

        static const uint8_t RANGER_ID = 121;  // VL53L5 ranger
        static const uint8_t MOCAP_ID  = 122;  // PAA3905 motion capture

        Msp m_msp;

        HardwareSerial * m_uart;

    public:

        SkyclientTask()
            : Task(SKYRANGER, 50) // Hz
        {
        }

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

            while (m_uart->available()) {

                Serial.println(m_uart->read(), HEX);

                /*
                auto messageType = m_msp.parse(m_uart->read());

                switch (messageType) {

                    case RANGER_ID: 
                        g_rangerCount++;
                        break;

                    case MOCAP_ID:
                        g_mocapCount++;
                        break;
                }*/
            }
        }

        void begin(HardwareSerial * uart)
        {
            m_uart = uart;
        }
};
