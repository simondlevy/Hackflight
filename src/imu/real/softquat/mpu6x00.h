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

#include <SPI.h>

#include <stdint.h>

#include "imu/real/softquat.h"

class Mpu6x00 : public SoftQuatImu {

    public:

        typedef enum {

            GYRO_250DPS,
            GYRO_500DPS,
            GYRO_1000DPS,
            GYRO_2000DPS

        } gyroScale_e;

        typedef enum {

            ACCEL_2G,
            ACCEL_4G,  
            ACCEL_8G,  
            ACCEL_16G

        } accelScale_e;

    private:

        // Registers
        static const uint8_t REG_SMPLRT_DIV    = 0x19;
        static const uint8_t REG_CONFIG        = 0x1A;
        static const uint8_t REG_GYRO_CONFIG   = 0x1B;
        static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
        static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
        static const uint8_t REG_INT_PIN_CFG   = 0x37;
        static const uint8_t REG_INT_ENABLE    = 0x38;
        static const uint8_t REG_USER_CTRL     = 0x6A;
        static const uint8_t REG_PWR_MGMT_1    = 0x6B;
        static const uint8_t REG_PWR_MGMT_2    = 0x6C;

        // Configuration bits  
        static const uint8_t BIT_RAW_RDY_EN       = 0x01;
        static const uint8_t BIT_CLK_SEL_PLLGYROZ = 0x03;
        static const uint8_t BIT_INT_ANYRD_2CLEAR = 0x10;
        static const uint8_t BIT_I2C_IF_DIS       = 0x10;
        static const uint8_t BIT_H_RESET          = 0x80;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        gyroScale_e m_gyroScale;

        accelScale_e m_accelScale;

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        uint8_t m_buffer[15];

        int16_t getValue(const uint8_t offset, const uint8_t index)
        {
            const uint8_t k = offset + index * 2;
            return (int16_t)(m_buffer[k] << 8 | m_buffer[k+1]);
        }

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

            readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14);

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(uint32_t clockSpeed) override
        {
            SoftQuatImu::begin(clockSpeed);

            m_spi.setBitOrder(MSBFIRST);
            m_spi.setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_INIT_CLK_HZ));
            m_spi.setDataMode(SPI_MODE3);

            // Chip reset
            writeRegister(REG_PWR_MGMT_1, BIT_H_RESET);
            delay(100);

            // Clock Source PPL with Z axis gyro reference
            writeRegister(REG_PWR_MGMT_1, BIT_CLK_SEL_PLLGYROZ);
            delayMicroseconds(7);

            // Disable Primary I2C Interface
            writeRegister(REG_USER_CTRL, BIT_I2C_IF_DIS);
            delayMicroseconds(15);

            writeRegister(REG_PWR_MGMT_2, 0x00);
            delayMicroseconds(15);

            // Accel Sample Rate 1kHz
            // Gyroscope Output Rate =  1kHz when the DLPF is enabled
            writeRegister(REG_SMPLRT_DIV, 0);
            delayMicroseconds(15);

            // Gyro +/- 2000 DPS Full Scale
            writeRegister(REG_GYRO_CONFIG, m_gyroScale << 3);
            delayMicroseconds(15);

            // Accel +/- 16 G Full Scale
            writeRegister(REG_ACCEL_CONFIG, m_accelScale << 3);
            delayMicroseconds(15);

            // INT_ANYRD_2CLEAR
            writeRegister(REG_INT_PIN_CFG, 0x10);

            delayMicroseconds(15);

            writeRegister(REG_INT_ENABLE, BIT_RAW_RDY_EN);
            delayMicroseconds(15);

            setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_CLK_HZ));
            delayMicroseconds(1);

            setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_INIT_CLK_HZ));

            // Accel and Gyro DLPF Setting
            writeRegister(REG_CONFIG, 0); // no gyro DLPF
            delayMicroseconds(1);

            setClockDivider(calculateSpiDivisor(clockSpeed, MAX_SPI_CLK_HZ));

        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            return getValue(9, k);
        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            return getValue(1, k);
        }

    public:

        Mpu6x00(
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

}; // class Mpu6x00
