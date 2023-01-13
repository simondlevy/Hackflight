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

        SPIClass * m_spi;

        // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
        // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
        uint8_t m_sampleRateDivisor;

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

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_csPin, LOW);
            m_spi->transfer(reg);
            m_spi->transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi->transfer(buffer, count+1);
            digitalWrite(m_csPin, HIGH);
        }

        uint8_t readRegister(const uint8_t addr)
        {
            uint8_t buffer[2] = {};
            readRegisters(addr, buffer, 1);
            return buffer[1];
        }

        void setClockDivider(uint32_t divider)
        {
            m_spi->setClockDivider(divider);
        }

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

        uint8_t m_csPin;

        uint16_t calculateSpiDivisor(const uint32_t clockSpeed, const uint32_t freq)
        {
            uint32_t clk = clockSpeed / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

        virtual bool gyroIsReady(void) override
        {

            // readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14);

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(uint32_t clockSpeed) override
        {
            pinMode(m_csPin, OUTPUT);
            pinMode(m_csPin, OUTPUT);

            m_spi->begin();

            writeRegister(REG_BANK_SEL, 0);

            writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

            delay(1);

            while (true) {
                Serial.println(readRegister(0x75), HEX);
            }

            SoftQuatImu::begin(clockSpeed);
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
                SPIClass & spi,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const uint8_t sampleRateDivisor = 19,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : SoftQuatImu(rotateFun, gyroScaleToInt(gyroScale), accelScaleToInt(accelScale))
        {
            m_spi = &spi;

            m_csPin = csPin;
            m_sampleRateDivisor = sampleRateDivisor;
            m_gyroScale = gyroScale;
            m_accelScale = accelScale;
        }

        void handleInterrupt(uint32_t cycleCounter)
        {
            (void)cycleCounter;
        }

}; // class Icm42688
