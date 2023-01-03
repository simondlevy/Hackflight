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
#include "debugger.h"

class SensorsTask : public Task {

    private:

        static const uint8_t RANGER_ID = 121;  // VL53L5 ranger
        static const uint8_t MOCAP_ID  = 122;  // PAA3905 motion capture

        Msp m_parser;

    public:

        int16_t mocapData[2];
        int16_t rangerData[16];

        SensorsTask()
            : Task(SENSORS, 50) // Hz
        {
        }

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

        }

        virtual void parse(const uint8_t byte)
        {
            auto messageType = m_parser.parse(byte);

            switch (messageType) {

                case 221: // VL53L5 ranger
                    for (uint8_t k=0; k<16; ++k) {
                        rangerData[k] = m_parser.parseShort(k);
                    }
                    break;

                case 222: // PAA3906 mocap
                    mocapData[0] = m_parser.parseShort(0);
                    mocapData[1] = m_parser.parseShort(1);
                    break;
            }
        }

};
