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
#include <Wire.h>

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

    protected:

        virtual void begin(void) override 
        {
            Wire.begin();
            Wire.setClock(400000); 
            delay(100);

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

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            // Simulates rocking in the X (phi) axis

            (void)time;

            static float phi;
            static int8_t dir = +1;

            phi += .01 * dir;

            if (phi >= 1.0) {
                dir = -1;
            }

            if (phi <= -1.0) {
                dir = +1;
            }

            return Axes(phi, 0.1, 0.1);
        }

        virtual uint32_t getGyroInterruptCount(void) override
        {
            static uint32_t _count;
            static uint32_t _tprev;

            // Simulate 8kHz interrupts
            uint32_t time = micros();
            if (time - _tprev > 125) {
                _count++;
                _tprev = time;
            }

            return _count;
        }

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles) override
        {
            (void)nextTargetCycles;
            (void)desiredPeriodCycles;
            return 0;
        }

        virtual bool gyroIsCalibrating(void) override
        {
            return false;
        }

        virtual bool gyroIsReady(void) override
        {
            return false;
        }

        virtual auto readGyroDps(const align_fun align) -> Axes  override
        {
            (void)align;
            return Axes(0, 0, 0);
        }

    public:

        void handleInterrupt(void)
        {
        }
};
