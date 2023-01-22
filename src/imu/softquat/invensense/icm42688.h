/*
   Class definition for ICM42688 IMU using SPI bus

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

class Icm42688 : public InvenSenseImu {

    public:

        typedef enum {

            ODR_8K = 3,  
            ODR_4K, 
            ODR_2K, 
            ODR_1K

        } odr_e;

    private:

        static const uint8_t REG_ACCEL_CONFIG_STATIC2 = 0x03;
        static const uint8_t REG_ACCEL_CONFIG_STATIC3 = 0x04;
        static const uint8_t REG_ACCEL_CONFIG_STATIC4 = 0x05;
        static const uint8_t REG_GYRO_CONFIG_STATIC3  = 0x0C;
        static const uint8_t REG_GYRO_CONFIG_STATIC4  = 0x0D;        
        static const uint8_t REG_GYRO_CONFIG_STATIC5  = 0x0E;
        static const uint8_t REG_INT_CONFIG           = 0x14;
        static const uint8_t REG_TEMP_DATA_A1         = 0x1D;
        static const uint8_t REG_PWR_MGMT0            = 0x4E;
        static const uint8_t REG_GYRO_CONFIG0         = 0x4F;
        static const uint8_t REG_ACCEL_CONFIG0        = 0x50;
        static const uint8_t REG_GYRO_ACCEL_CONFIG0   = 0x52;
        static const uint8_t REG_INT_CONFIG0          = 0x63;
        static const uint8_t REG_INT_CONFIG1          = 0x64;
        static const uint8_t REG_INT_SOURCE0          = 0x65;
        static const uint8_t REG_BANK_SEL             = 0x76;

        static const uint8_t PWR_MGMT0_ACCEL_MODE_LN    = 3 << 0;
        static const uint8_t PWR_MGMT0_GYRO_MODE_LN     = 3 << 2;
        static const uint8_t PWR_MGMT0_TEMP_DISABLE_OFF = 0 << 5;

        static const uint8_t ACCEL_UI_FILT_BW_LOW_LATENCY = 14 << 4;
        static const uint8_t GYRO_UI_FILT_BW_LOW_LATENCY  = 14 << 0;

        static const uint8_t INT1_MODE_PULSED          = 0 << 2;
        static const uint8_t INT1_DRIVE_CIRCUIT_PP     = 1 << 1;
        static const uint8_t INT1_POLARITY_ACTIVE_HIGH = 1 << 0;

        static const uint8_t INT_ASYNC_RESET_BIT       = 4;
        static const uint8_t INT_TDEASSERT_DISABLE_BIT = 5;
        static const uint8_t INT_TPULSE_DURATION_BIT   = 6;

        static const uint8_t INT_TPULSE_DURATION_8  = 1 << INT_TPULSE_DURATION_BIT;
        static const uint8_t INT_TDEASSERT_DISABLED = 1 << INT_TDEASSERT_DISABLE_BIT;

        static const uint8_t UI_DRDY_INT_CLEAR_ON_SBR  = (0 << 5) || (0 << 4);
        static const uint8_t UI_DRDY_INT1_EN_ENABLED  = 1 << 3;

        static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;

        static const uint32_t MAX_SPI_CLOCK_RATE = 24000000;

        odr_e   m_odr;
        uint8_t m_antiAliasDelta;
        uint8_t m_antiAliasBitshift;

        static uint16_t gyroScaleToInt(const gyroScale_e gyroScale)
        {
            return
                gyroScale == GYRO_250DPS ?  250 : 
                gyroScale == GYRO_500DPS ?  500 : 
                gyroScale == GYRO_1000DPS ?  1000 : 
                2000;
        }

        static uint16_t accelScaleToInt(const accelScale_e accelScale)
        {
            return
                accelScale == ACCEL_2G ?  2 : 
                accelScale == ACCEL_4G ?  4 : 
                accelScale == ACCEL_8G ?  8 : 
                16;
        }

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

        virtual void getRegisterSettings(
                std::vector<registerSetting_t> & settings) override
        {
            settings.push_back({REG_BANK_SEL, 0});

            settings.push_back({UB0_REG_DEVICE_CONFIG, 0x01});

            settings.push_back({REG_PWR_MGMT0,
                    PWR_MGMT0_TEMP_DISABLE_OFF |
                    PWR_MGMT0_ACCEL_MODE_LN |
                    PWR_MGMT0_GYRO_MODE_LN});

            settings.push_back({REG_GYRO_CONFIG0,
                    (3 - GYRO_2000DPS) << 5 | (m_odr & 0x0F)});

            settings.push_back({REG_ACCEL_CONFIG0,
                    (3 - ACCEL_16G) << 5 | (m_odr & 0x0F)});

            settings.push_back({REG_GYRO_CONFIG_STATIC3, m_antiAliasDelta});

            uint16_t deltSqr = m_antiAliasDelta * m_antiAliasDelta;
            settings.push_back({REG_GYRO_CONFIG_STATIC4, deltSqr & 0xFF});

            settings.push_back({REG_GYRO_CONFIG_STATIC5,
                    (deltSqr >> 8) | (m_antiAliasBitshift << 4)});

            settings.push_back({REG_ACCEL_CONFIG_STATIC2, m_antiAliasDelta << 1});

            settings.push_back({REG_ACCEL_CONFIG_STATIC3, deltSqr & 0xFF});

            settings.push_back({REG_ACCEL_CONFIG_STATIC4,
                    (deltSqr >> 8) | (m_antiAliasBitshift << 4)});

            settings.push_back({REG_GYRO_ACCEL_CONFIG0,
                    ACCEL_UI_FILT_BW_LOW_LATENCY | GYRO_UI_FILT_BW_LOW_LATENCY});

            settings.push_back({REG_INT_CONFIG,
                    INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PP | INT1_POLARITY_ACTIVE_HIGH});

            settings.push_back({REG_INT_CONFIG0, UI_DRDY_INT_CLEAR_ON_SBR});

            settings.push_back({REG_INT_SOURCE0, UI_DRDY_INT1_EN_ENABLED});

            // Datasheet says: "User should change setting to 0 from default
            // setting of 1, for proper INT1 and INT2 pin operation"
            //uint8_t intConfig1Value = readRegister(REG_INT_CONFIG1);
            //intConfig1Value &= ~(1 << INT_ASYNC_RESET_BIT);
            //intConfig1Value |= (INT_TPULSE_DURATION_8 | INT_TDEASSERT_DISABLED);
            //settings.push_back({REG_INT_CONFIG1, intConfig1Value});
        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            // Accel data is second value, after temperature
            return getShortFromBuffer(1, k);
        }

    public:

        Icm42688(
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G,
                const odr_e odr = ODR_8K,
                const uint8_t antiAliasDelta = 6,
                const uint8_t antiAliasBitshift = 10)
            : InvenSenseImu(
                    MAX_SPI_CLOCK_RATE,
                    MAX_SPI_CLOCK_RATE,
                    mosiPin,
                    misoPin,
                    sclkPin,
                    csPin,
                    REG_TEMP_DATA_A1,
                    rotateFun,
                    gyroScale,
                    accelScale)
    {
        m_odr = odr;
        m_antiAliasDelta = antiAliasDelta;
        m_antiAliasBitshift = antiAliasBitshift;
    }

}; // class Icm42688
