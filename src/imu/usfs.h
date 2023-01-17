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

class UsfsImu : public Imu {

    private:

        static const uint8_t  GYRO_RATE_TENTH = 100;   // 1/10th actual rate
        static const uint16_t GYRO_SCALE_DPS  = 2000;

        // Arbitrary; unused
        static const uint8_t  ACCEL_BANDWIDTH  = 3;
        static const uint8_t  GYRO_BANDWIDTH   = 3;
        static const uint8_t  QUAT_DIVISOR     = 1;
        static const uint8_t  MAG_RATE         = 100;
        static const uint8_t  ACCEL_RATE_TENTH = 20; // Multiply by 10 to get actual rate
        static const uint8_t  BARO_RATE        = 50;
        static const uint16_t ACCEL_SCALE      = 8;
        static const uint16_t MAG_SCALE        = 1000;

        static const uint8_t INTERRUPT_ENABLE = USFS_INTERRUPT_RESET_REQUIRED |
            USFS_INTERRUPT_ERROR |
            USFS_INTERRUPT_GYRO | 
            USFS_INTERRUPT_QUAT;

        bool     m_gotNewData;
        uint32_t m_gyroInterruptCount;
        uint32_t m_gyroSyncTime;

        float m_qw;
        float m_qx;
        float m_qy;
        float m_qz;

        int16_t m_gyroAdc[3];

    protected:

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            (void)time;

            Axes angles = quat2euler(m_qw, m_qx, m_qy, m_qz);

            return Axes(angles.x, -angles.y, -angles.z);
        }

        virtual uint32_t getGyroInterruptCount(void) override
        {
            return m_gyroInterruptCount;
        }

        virtual bool gyroIsReady(void) override
        {
            bool result = false;

            if (m_gotNewData) { 

                m_gotNewData = false;  

                uint8_t eventStatus = usfsCheckStatus(); 

                if (usfsEventStatusIsError(eventStatus)) { 
                    usfsReportError(eventStatus);
                }

                if (usfsEventStatusIsGyrometer(eventStatus)) { 
                    usfsReadGyrometerRaw(m_gyroAdc);
                    result = true;
                }

                if (usfsEventStatusIsQuaternion(eventStatus)) { 
                    usfsReadQuaternion(m_qw, m_qx, m_qy, m_qz);
                }
            } 

            return result;
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            return m_gyroAdc[k];
        }

    public:

        UsfsImu(rotateFun_t rotateFun) 
            : Imu(rotateFun, 1.53e-1) // gyro rate fixed in master mode
        {
        }

        void begin(void)
        {
            usfsLoadFirmware(); 

            usfsBegin(
                    ACCEL_BANDWIDTH,
                    GYRO_BANDWIDTH,
                    QUAT_DIVISOR,
                    MAG_RATE,
                    ACCEL_RATE_TENTH,
                    GYRO_RATE_TENTH,
                    BARO_RATE,
                    INTERRUPT_ENABLE);

            // Clear interrupts
            usfsCheckStatus();
        }

        void handleInterrupt(const uint32_t usec)
        {
            m_gotNewData = true;
            m_gyroInterruptCount++;
            m_gyroSyncTime = usec;
        }
};
