/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

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

#include <stdint.h>

#include "imu/softquat/invensense.h"

class Mpu6000 : public InvenSenseImu {

    private:

        static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

        static const uint8_t ACCEL_BUFFER_OFFSET = 0;
        static const uint8_t GYRO_BUFFER_OFFSET = 4;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;


    public:

        Mpu6000(
                const rotateFun_t rotateFun,
                const uint8_t csPin,
                const gyroFsr_e gyroFsr = GYRO_2000DPS,
                const accelFsr_e accelFsr = ACCEL_16G)
            : InvenSenseImu(
                    csPin,
                    MAX_SPI_INIT_CLK_HZ,
                    MAX_SPI_CLK_HZ,
                    REG_ACCEL_XOUT_H,
                    ACCEL_BUFFER_OFFSET,
                    GYRO_BUFFER_OFFSET,
                    rotateFun,
                    gyroFsr,
                    accelFsr)
    {
    }

}; // class Mpu6000
