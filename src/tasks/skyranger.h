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

class SkyrangerTask : public Task {

    private:

        static const uint8_t MSP_SET_ATTITUDE = 213;
        static const uint8_t MSP_SET_VL53L5   = 221;
        static const uint8_t MSP_SET_PAA3905  = 222;

        Msp m_parser;
        Msp m_serializer;

        VehicleState * m_vstate;

    public:

        int16_t mocapData[2];
        int16_t rangerData[16];

        SkyrangerTask(VehicleState & vstate)
            : Task(SKYRANGER, 50) // Hz
        {
            m_vstate = &vstate;
        }

        virtual void run(const uint32_t usec) override
        {
            (void)usec;

            int16_t angles[3] = {};
            Imu::getEulerAngles(m_vstate, angles);

            m_serializer.serializeShorts(MSP_SET_ATTITUDE, angles, 3);
        }

        virtual void parse(const uint8_t byte)
        {
            switch (m_parser.parse(byte)) {

                case MSP_SET_VL53L5: 
                    for (uint8_t k=0; k<16; ++k) {
                        rangerData[k] = m_parser.parseShort(k);
                    }
                    break;

                case MSP_SET_PAA3905:
                    mocapData[0] = m_parser.parseShort(0);
                    mocapData[1] = m_parser.parseShort(1);
                    break;
            }
        }

        uint8_t imuDataAvailable(void)
        {
            return m_serializer.dataAvailable();
        }

        uint8_t readImuData(void)
        {
            return m_serializer.read();
        }

}; // class SkyrangerTask
