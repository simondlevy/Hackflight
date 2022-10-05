/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY without even the implied warranty of MERCHANTABILITY or FITNESS
   FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
   details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <imu.h>
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

        static const uint8_t REPORT_HZ = 2;

        uint8_t m_interruptPin;

        int16_t m_gyroAdc[3];

        float m_qw, m_qx, m_qy, m_qz;

    protected:

        virtual auto getEulerAngles(const uint32_t time) -> Axes override;

        virtual bool devGyroIsReady(void) override;

        virtual void devInit(
                uint32_t * gyroSyncTimePtr, uint32_t * gyroInterruptCountPtr) override;

        virtual int16_t devReadRawGyro(uint8_t k) override;

        static void interruptHandler(void);

    public:

        // Shared with interrupt handler routine
        typedef struct {

            bool       gotNewData;
            uint32_t * syncTimePtr;
            uint32_t * interruptCountPtr;

        } gyroDev_t;

        UsfsImu(uint8_t interruptPin);
};
