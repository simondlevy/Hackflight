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

#include <SPI.h>

#include <stdint.h>

#include "imu/real/invensense.h"

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

        virtual bool gyroIsReady(void) override
        {
             readRegisters(REG_TEMP_DATA_A1);

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(uint32_t clockSpeed) override
        {
            InvenSenseImu::begin(clockSpeed);

            writeRegister(REG_BANK_SEL, 0);

            writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

            delay(1);

            m_spi.setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_CLOCK_RATE));

            writeRegister(REG_PWR_MGMT0,
                    PWR_MGMT0_TEMP_DISABLE_OFF |
                    PWR_MGMT0_ACCEL_MODE_LN |
                    PWR_MGMT0_GYRO_MODE_LN);
            delay(15);

            writeRegister(REG_GYRO_CONFIG0, (3 - GYRO_2000DPS) << 5 | (m_odr & 0x0F));
            delay(15);

            writeRegister(REG_ACCEL_CONFIG0, (3 - ACCEL_16G) << 5 | (m_odr & 0x0F));
            delay(15);

            // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
            writeRegister(REG_GYRO_CONFIG_STATIC3, m_antiAliasDelta);
            uint16_t deltSqr = m_antiAliasDelta * m_antiAliasDelta;
            writeRegister(REG_GYRO_CONFIG_STATIC4, deltSqr & 0xFF);
            writeRegister(REG_GYRO_CONFIG_STATIC5,
                    (deltSqr >> 8) | (m_antiAliasBitshift << 4));

            // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
            writeRegister(REG_ACCEL_CONFIG_STATIC2, m_antiAliasDelta << 1);
            writeRegister(REG_ACCEL_CONFIG_STATIC3, deltSqr & 0xFF);
            writeRegister(REG_ACCEL_CONFIG_STATIC4,
                    (deltSqr >> 8) | (m_antiAliasBitshift << 4));

            // Configure gyro and acc UI Filters
            writeRegister(REG_GYRO_ACCEL_CONFIG0,
                    ACCEL_UI_FILT_BW_LOW_LATENCY | GYRO_UI_FILT_BW_LOW_LATENCY);
            writeRegister(REG_INT_CONFIG,
                    INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PP | INT1_POLARITY_ACTIVE_HIGH);
            writeRegister(REG_INT_CONFIG0, UI_DRDY_INT_CLEAR_ON_SBR);
            writeRegister(REG_INT_SOURCE0, UI_DRDY_INT1_EN_ENABLED);

            // Datasheet says: "User should change setting to 0 from default
            // setting of 1, for proper INT1 and INT2 pin operation"
            uint8_t intConfig1Value = readRegister(REG_INT_CONFIG1);
            intConfig1Value &= ~(1 << INT_ASYNC_RESET_BIT);
            intConfig1Value |= (INT_TPULSE_DURATION_8 | INT_TDEASSERT_DISABLED);
            writeRegister(REG_INT_CONFIG1, intConfig1Value);
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            // Gyro data is fifth value
            return getShortFromBuffer(4, k);
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
                    mosiPin, misoPin, sclkPin, csPin, rotateFun, gyroScale, accelScale)
    {
        m_odr = odr;
        m_antiAliasDelta = antiAliasDelta;
        m_antiAliasBitshift = antiAliasBitshift;
    }

}; // class Icm42688
