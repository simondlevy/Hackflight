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

#include <string.h>

#include <mpu6x00.h>

#include "imu/softquat.h"

class Mpu6000 : public SoftQuatImu {

    friend class Stm32FBoard;

    private:

        static uint16_t gyroFsrToInt(const Mpu6x00::gyroFsr_e gyroFsr)
        {
            return
                gyroFsr == Mpu6x00::GYRO_250DPS ?  250 : 
                gyroFsr == Mpu6x00::GYRO_500DPS ?  500 : 
                gyroFsr == Mpu6x00::GYRO_1000DPS ?  1000 : 
                2000;
        }

        static uint16_t accelFsrToInt(const Mpu6x00::accelFsr_e accelFsr)
        {
            return
                accelFsr == Mpu6x00::ACCEL_2G ?  2 : 
                accelFsr == Mpu6x00::ACCEL_4G ?  4 : 
                accelFsr == Mpu6x00::ACCEL_8G ?  8 : 
                16;
        }

        Mpu6x00 * m_mpu;

    protected:

        virtual int16_t readRawAccel(uint8_t k) override
        {
            (void)k;
            return 0; // XXX
        }

        void begin(const uint32_t mcuClockSpeed) override
        {
            SoftQuatImu::begin(mcuClockSpeed);

            m_mpu->begin();
        }

        virtual bool gyroIsReady(void)  override
        {
            return true;
        }

        virtual void getRawGyro(int16_t rawGyro[3]) override
        {
            m_mpu->readData();

            m_mpu->getRawGyro(rawGyro[0], rawGyro[1], rawGyro[2]);
        }

    public:

        Mpu6000(
                Mpu6x00 & mpu,
                const rotateFun_t rotateFun,
                const Mpu6x00::gyroFsr_e gyroFsr = Mpu6x00::GYRO_2000DPS,
                const Mpu6x00::accelFsr_e accelFsr = Mpu6x00::ACCEL_16G)
            : SoftQuatImu(rotateFun, gyroFsrToInt(gyroFsr), accelFsrToInt(accelFsr))
        {
            m_mpu = &mpu;
        }

};
