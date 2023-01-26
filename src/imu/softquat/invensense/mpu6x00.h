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

class Mpu6x00 : public InvenSenseImu {

    private:

        static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

        static const uint8_t ACCEL_BUFFER_OFFSET = 0;
        static const uint8_t GYRO_BUFFER_OFFSET = 4;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

    protected:

        virtual void initRegisters(void) override
        {
            writeRegister(REG_PWR_MGMT_1, BIT_RESET);
            delay(100);

            writeRegister(REG_PWR_MGMT_1, BIT_CLK_SEL_PLLGYROZ);
            delayMicroseconds(7);

            writeRegister(REG_USER_CTRL, BIT_I2C_IF_DIS);
            delayMicroseconds(15);

            writeRegister(REG_PWR_MGMT_2, 0x00);
            delayMicroseconds(15);

            writeRegister(REG_SMPLRT_DIV, 0);
            delayMicroseconds(15);

            writeRegister(REG_GYRO_CONFIG, (uint8_t)(m_gyroFsr << 3));
            delayMicroseconds(15);

            writeRegister(REG_ACCEL_CONFIG, (uint8_t)(m_accelFsr << 3));
            delayMicroseconds(15);

            writeRegister(REG_INT_PIN_CFG, 0x10);
            delayMicroseconds(15);

            writeRegister(REG_INT_ENABLE, BIT_RAW_RDY_EN);
            delayMicroseconds(15);

            writeRegister(REG_CONFIG, 0);
            delayMicroseconds(1);
        }

    public:

        Mpu6x00(
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

}; // class Mpu6x00
