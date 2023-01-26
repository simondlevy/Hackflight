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

#include "imu.h"

#include <USFS.h>

class LadybugImu : public Imu {

    friend class LadybugBoard;

    private:

        // Arbitrary; unused
        static const uint8_t  ACCEL_BANDWIDTH  = 3;
        static const uint8_t  GYRO_BANDWIDTH   = 3;
        static const uint8_t  QUAT_DIVISOR     = 1;
        static const uint8_t  MAG_RATE         = 100;
        static const uint8_t  ACCEL_RATE_TENTH = 20; // Multiply by 10 to get actual rate
        static const uint8_t  BARO_RATE        = 50;
        static const uint16_t ACCEL_SCALE      = 8;
        static const uint16_t MAG_SCALE        = 1000;

        static const uint16_t GYRO_SCALE_DPS  = 2000;

        static const uint8_t  GYRO_RATE_TENTH = 100;   // 1/10th actual rate

        static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
            Usfs::INTERRUPT_ERROR |
            Usfs::INTERRUPT_GYRO | 
            Usfs::INTERRUPT_QUAT;

        float m_qw;
        float m_qx;
        float m_qy;
        float m_qz;

        bool m_gotNewData;

        Usfs usfs;

        int16_t m_rawGyro[3];

        LadybugImu(void) 
            : Imu(Imu::rotate0, GYRO_SCALE_DPS)
        {
        }

        void begin(const uint32_t clockSpeed) override
        {
            (void)clockSpeed;

            usfs.loadFirmware(); 

            usfs.begin(
                    ACCEL_BANDWIDTH,
                    GYRO_BANDWIDTH,
                    QUAT_DIVISOR,
                    MAG_RATE,
                    ACCEL_RATE_TENTH,
                    GYRO_RATE_TENTH,
                    BARO_RATE,
                    INTERRUPT_ENABLE);

            // Clear interrupts
            Usfs::checkStatus();

            setGyroCalibrationCycles();
        }

    public:

        virtual bool gyroIsReady(void) override
        {
            bool result = false;

            if (m_gotNewData) { 

                m_gotNewData = false;  

                uint8_t eventStatus = Usfs::checkStatus(); 

                if (Usfs::eventStatusIsError(eventStatus)) { 
                    Usfs::reportError(eventStatus);
                }

                if (Usfs::eventStatusIsGyrometer(eventStatus)) { 
                    usfs.readGyrometerRaw(m_rawGyro);
                    result = true;
                }

                if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
                    usfs.readQuaternion(m_qw, m_qx, m_qy, m_qz);
                }
            } 

            return result;
        }

        virtual void getRawGyro(int16_t rawGyro[3]) override
        {
            rawGyro[0] = m_rawGyro[0];
            rawGyro[1] = m_rawGyro[1];
            rawGyro[2] = m_rawGyro[2];
        }

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            (void)time;

            Axes angles = quat2euler(m_qw, m_qx, m_qy, m_qz);

            return Axes(angles.x, -angles.y, -angles.z);
        }

        void handleInterrupt(const uint32_t cycleCounter)
        {
            (void)cycleCounter;

            m_gotNewData = true;
        }
};
