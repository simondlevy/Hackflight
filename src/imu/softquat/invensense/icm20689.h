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

    public:


    private:

        static const uint8_t TEMP_RST  = 0x01;
        static const uint8_t ACCEL_RST = 0x02;
        static const uint8_t BIT_RESET  = 0x80;
        static const uint8_t I2C_IF_DIS = 0x10;

        static const uint32_t MAX_SPI_CLK_HZ 8000000;

        virtual void getRegisterSettings(
                std::vector<registerSetting_t> & settings) override
        {
            settings.push_back(REG_PWR_MGMT_1, BIT_RESET);
            settings.push_back(REG_USER_CTRL, I2C_IF_DIS);
            settings.push_back(REG_SIGNAL_PATH_RESET, ACCEL_RST | TEMP_RST);
            settings.push_back(REG_PWR_MGMT_1, (uint8_t)INV_CLK_PLL);

            settings.push_back(REG_GYRO_CONFIG, INV_FSR_2000DPS << 3);
            settings.push_back(REG_ACCEL_CONFIG, INV_FSR_16G << 3);

            settings.push_back(REG_CONFIG, mpuGyroDLPF(gyro));
            settings.push_back(REG_SMPLRT_DIV, gyro->mpuDividerDrops);
            settings.push_back(REG_INT_PIN_CFG, ICM20689_INT_ANYRD_2CLEAR);
            settings.push_back(REG_INT_ENABLE, MPU_RF_DATA_RDY_EN);

        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            // Accel data is first value in buffer
            return getShortFromBuffer(0, k);
        }


    public:

        Icm20689(
                const uint8_t csPin,
                const rotateFun_t rotateFun)
            : InvenSenseImu(
                    csPin,
                    MAX_SPI_CLOCK_HZ,
                    MAX_SPI_CLOCK_HZ,
                    REG_TEMP_DATA_A1,
                    rotateFun,
                    gyroScale,
                    accelScale)
    {
    }

}; // class Icm20689
