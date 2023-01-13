/*
   Class definition for ICM42688 IMU using SPI bus

   Adapted from https://github.com/finani/ICM42688

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

#include "imu/real/softquat.h"

class Icm42688 : public SoftQuatImu {

    public:

        // No need to support gyro rates below 250
        typedef enum {

            GYRO_2000DPS,
            GYRO_1000DPS,
            GYRO_500DPS,
            GYRO_250DPS

        } gyroScale_e;

        typedef enum {

            ACCEL_16G,
            ACCEL_4G,  
            ACCEL_8G,  
            ACCEL_2G

        } accelScale_e;

        typedef enum {

            ODR_32K, // LN mode only
            ODR_16K, // LN mode only
            ODR_8K,  // LN mode only
            ODR_4K,  // LN mode only
            ODR_2K,  // LN mode only
            ODR_1K,  // LN mode only
            ODR_200,
            ODR_100,
            ODR_50,
            ODR_25,
            ODR_12_5,
            ODR_6a25,   // LP mode only (accel only)
            ODR_3a125,  // LP mode only (accel only)
            ODR_1a5625, // LP mode only (accel only)
            ODR_500
        };

    private:

        static constexpr uint8_t REG_BANK_SEL          = 0x76;
        static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;

        static const uint32_t MAX_SPI_CLOCK_RATE = 24000000;

        gyroScale_e m_gyroScale;

        accelScale_e m_accelScale;

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

        static float scale(const uint16_t n)
        {
            return n / 32768.0;
        }

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

        virtual bool gyroIsReady(void) override
        {

            // readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14);

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(uint32_t clockSpeed) override
        {
            SoftQuatImu::begin(clockSpeed);

            writeRegister(REG_BANK_SEL, 0);

            writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

            delay(1);

            m_spi.setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_CLOCK_RATE));

            /*
            mpuGyroInit(gyro);
            gyro->accDataReg = ICM426XX_RA_ACCEL_DATA_X1;
            gyro->gyroDataReg = ICM426XX_RA_GYRO_DATA_X1;

            spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
            delay(15);

            // Get desired output data rate
            uint8_t odrConfig;
            const unsigned decim = llog2(gyro->mpuDividerDrops + 1);
            if (gyro->gyroRateKHz && decim < ODR_CONFIG_COUNT) {
                odrConfig = odrLUT[decim];
            } else {
                odrConfig = odrLUT[ODR_CONFIG_1K];
                gyro->gyroRateKHz = GYRO_RATE_1_kHz;
            }

            STATIC_ASSERT(INV_FSR_2000DPS == 3, "INV_FSR_2000DPS must be 3 to generate correct value");
            spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG0, (3 - INV_FSR_2000DPS) << 5 | (odrConfig & 0x0F));
            delay(15);

            STATIC_ASSERT(INV_FSR_16G == 3, "INV_FSR_16G must be 3 to generate correct value");
            spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG0, (3 - INV_FSR_16G) << 5 | (odrConfig & 0x0F));
            delay(15);

            // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
            aafConfig_t aafConfig = getGyroAafConfig();
            spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
            spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
            spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

            // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
            aafConfig = aafLUT[AAF_CONFIG_258HZ];
            spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
            spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
            spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

            // Configure gyro and acc UI Filters
            spiWriteReg(dev, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

            spiWriteReg(dev, ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
            spiWriteReg(dev, ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);

            spiWriteReg(dev, ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

            uint8_t intConfig1Value = spiReadRegMsk(dev, ICM426XX_RA_INT_CONFIG1);
            // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
            intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
            intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

            spiWriteReg(dev, ICM426XX_RA_INT_CONFIG1, intConfig1Value);*/

        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            (void)k;
            return 0;
        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            (void)k;
            return 0;
        }

    public:

        Icm42688(
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : SoftQuatImu(
                    mosiPin,
                    misoPin,
                    sclkPin,
                    csPin,
                    rotateFun,
                    gyroScaleToInt(gyroScale),
                    accelScaleToInt(accelScale))
    {
        m_gyroScale = gyroScale;
        m_accelScale = accelScale;
    }

}; // class Icm42688
