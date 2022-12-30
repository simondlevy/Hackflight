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
#include "msp/uart.h"

class SkyrangerTask : public Task {

    private:

        static const uint8_t RANGER_ID = 121;  // VL53L5 ranger
        static const uint8_t MOCAP_ID  = 122;  // PAA3905 motion capture

        UartMsp m_msp;

        void sendRangerRequest(void)
        {
            m_msp.sendRequest(RANGER_ID);
        }

        void sendMocapRequest(void)
        {
            m_msp.sendRequest(MOCAP_ID);
        }

    public:

        SkyrangerTask()
            : Task(SKYRANGER, 50) // Hz
        {
        }

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

            /*
            while (m_msp.available()) {

                auto byte = m_msp.read();

                auto messageType = m_msp.parse(byte);

                switch (messageType) {

                    case RANGER_ID: 
                        break;

                    case MOCAP_ID:
                        break;

                }
            }*/
        }

        void begin(HardwareSerial * uart)
        {
            m_msp.begin(uart);

            sendRangerRequest();
            sendMocapRequest();
        }
};
