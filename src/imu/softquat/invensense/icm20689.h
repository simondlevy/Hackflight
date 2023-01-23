/*
   Class definition for ICM20689 IMU using SPI bus

   Copyright (c) 2023 Simon D. Levy

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

#include <stdint.h>

#include "imu/softquat/invensense.h"

class Icm20689 : public InvenSenseImu {

    private:

        static const uint8_t ACCEL_BUFFER_OFFSET = 0;
        static const uint8_t GYRO_BUFFER_OFFSET = 4;

        static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

        static const uint8_t TEMP_RST         = 0x01;
        static const uint8_t ACCEL_RST        = 0x02;
        static const uint8_t INT_ANYRD_2CLEAR = 0x10;

        static const uint32_t MAX_SPI_CLK_HZ = 8000000;

        virtual void getRegisterSettings(
                std::vector<registerSetting_t> & settings) override
        {
            settings.push_back({REG_PWR_MGMT_1, BIT_RESET});

            settings.push_back({REG_USER_CTRL, BIT_I2C_IF_DIS});

            settings.push_back({REG_SIGNAL_PATH_RESET, ACCEL_RST | TEMP_RST});

            settings.push_back({REG_PWR_MGMT_1, (uint8_t)INV_CLK_PLL});

            settings.push_back({REG_GYRO_CONFIG, (uint8_t)(m_gyroFsr << 3)});
            settings.push_back({REG_ACCEL_CONFIG, (uint8_t)(m_accelFsr << 3)});

            settings.push_back({REG_CONFIG, GYRO_HARDWARE_LPF_NORMAL});

            settings.push_back({REG_SMPLRT_DIV, 0x00});

            settings.push_back({REG_INT_PIN_CFG, INT_ANYRD_2CLEAR});

            settings.push_back({REG_INT_ENABLE, BIT_RAW_RDY_EN});
        }

    public:

        Icm20689(
                const rotateFun_t rotateFun,
                const uint8_t csPin,
                const gyroFsr_e gyroFsr = GYRO_2000DPS,
                const accelFsr_e accelFsr = ACCEL_16G)
            : InvenSenseImu(
                    csPin,
                    MAX_SPI_CLK_HZ,
                    MAX_SPI_CLK_HZ,
                    REG_ACCEL_XOUT_H,
                    ACCEL_BUFFER_OFFSET,
                    GYRO_BUFFER_OFFSET,
                    rotateFun,
                    gyroFsr,
                    accelFsr)
    {
    }

}; // class Icm20689
