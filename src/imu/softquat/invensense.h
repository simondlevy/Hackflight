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

#include "core/axes.h"
#include "core/clock.h"
#include "core/filters/pt2.h"
#include "core/vstate.h"
#include "imu/softquat.h"

class InvenSenseImu : public SoftQuatImu {

    friend class Stm32FBoard;

    public:

        typedef enum {

            GYRO_250DPS,
            GYRO_500DPS,
            GYRO_1000DPS,
            GYRO_2000DPS

        } gyroFsr_e;

        typedef enum {

            ACCEL_2G,
            ACCEL_4G,  
            ACCEL_8G,  
            ACCEL_16G

        } accelFsr_e;

        typedef enum {
            INV_CLK_INTERNAL,
            INV_CLK_PLL,
            NUM_CLK
        } clockSel_e;

        // Configuration bits  
        static const uint8_t BIT_RAW_RDY_EN       = 0x01;
        static const uint8_t BIT_CLK_SEL_PLLGYROZ = 0x03;
        static const uint8_t BIT_I2C_IF_DIS       = 0x10;
        static const uint8_t BIT_RESET            = 0x80;

        static const uint8_t GYRO_HARDWARE_LPF_NORMAL = 0x00;

    private:

        static const uint8_t IMU_MOSI_PIN = PA7;
        static const uint8_t IMU_MISO_PIN = PA6;
        static const uint8_t IMU_SCLK_PIN = PA5;

        static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

        static const uint8_t ACCEL_BUFFER_OFFSET = 0;
        static const uint8_t GYRO_BUFFER_OFFSET = 4;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t SPI_INIT_CLK_HZ = 1000000;

        uint8_t  m_csPin;

        SPIClass m_spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

        static uint16_t gyroFsrToInt(const gyroFsr_e gyroFsr)
        {
            return
                gyroFsr == GYRO_250DPS ?  250 : 
                gyroFsr == GYRO_500DPS ?  500 : 
                gyroFsr == GYRO_1000DPS ?  1000 : 
                2000;
        }

        static uint16_t accelFsrToInt(const accelFsr_e accelFsr)
        {
            return
                accelFsr == ACCEL_2G ?  2 : 
                accelFsr == ACCEL_4G ?  4 : 
                accelFsr == ACCEL_8G ?  8 : 
                16;
        }

        static uint16_t calculateSpiDivisor(const uint32_t clockSpeed, const uint32_t freq)
        {
            uint32_t clk = clockSpeed / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

    protected:

        // Registers
        static const uint8_t REG_SMPLRT_DIV        = 0x19;
        static const uint8_t REG_CONFIG            = 0x1A;
        static const uint8_t REG_GYRO_CONFIG       = 0x1B;
        static const uint8_t REG_ACCEL_CONFIG      = 0x1C;
        static const uint8_t REG_INT_PIN_CFG       = 0x37;
        static const uint8_t REG_INT_ENABLE        = 0x38;
        static const uint8_t REG_SIGNAL_PATH_RESET = 0x6A;
        static const uint8_t REG_USER_CTRL         = 0x6A;
        static const uint8_t REG_PWR_MGMT_1        = 0x6B;
        static const uint8_t REG_PWR_MGMT_2        = 0x6C;

        gyroFsr_e m_gyroFsr;
        accelFsr_e m_accelFsr;

        InvenSenseImu * m_invenSenseImu;

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        uint8_t m_buffer[15];

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            m_spi.beginTransaction(SPISettings(SPI_INIT_CLK_HZ, MSBFIRST, SPI_MODE3)); 

            digitalWrite(m_csPin, LOW);
            m_spi.transfer(reg);
            m_spi.transfer(val);
            digitalWrite(m_csPin, HIGH);

            m_spi.endTransaction(); 
        }

        void readRegisters(
                const uint8_t addr,
                uint8_t * buffer,
                const uint8_t count,
                const uint32_t spiClkHz) 
        {
            m_spi.beginTransaction(SPISettings(spiClkHz, MSBFIRST, SPI_MODE3)); 

            digitalWrite(m_csPin, LOW);

            buffer[0] = addr | 0x80;
            m_spi.transfer(buffer, count+1);

            digitalWrite(m_csPin, HIGH);

            m_spi.endTransaction(); 
        }

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        static const uint8_t BUFFER_SIZE = 15;

        int16_t getShortFromBuffer(const uint8_t offset, const uint8_t index)
        {
            const uint8_t k = 2 * (offset + index) + 1;
            return (int16_t)(m_buffer[k] << 8 | m_buffer[k+1]);
        }

        int16_t getGyroValFromBuffer(uint8_t k)
        {
            return getShortFromBuffer(GYRO_BUFFER_OFFSET, k);
        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            // Accel data is first value in buffer
            return getShortFromBuffer(ACCEL_BUFFER_OFFSET, k);
        }

        InvenSenseImu(
                const uint8_t csPin,
                const rotateFun_t rotateFun = rotate0,
                const gyroFsr_e gyroFsr = GYRO_2000DPS,
                const accelFsr_e accelFsr = ACCEL_16G)
            : SoftQuatImu(rotateFun, gyroFsrToInt(gyroFsr), accelFsrToInt(accelFsr))
        {
            m_csPin = csPin;

            m_gyroFsr = gyroFsr;
            m_accelFsr = accelFsr;
        }

        virtual void initRegisters(void)
        {
        }

        void begin(const uint32_t mcuClockSpeed) override
        {
            SoftQuatImu::begin(mcuClockSpeed);

            m_spi.begin();

            pinMode(m_csPin, OUTPUT);

            digitalWrite(m_csPin, HIGH);

            initRegisters();
        }

        virtual bool gyroIsReady(void)  override
        {
            return true;
        }

        virtual void getRawGyro(int16_t rawGyro[3]) override
        {
            readRegisters(
                    REG_ACCEL_XOUT_H,
                    m_buffer,
                    InvenSenseImu::BUFFER_SIZE,
                    MAX_SPI_CLK_HZ);

            rawGyro[0] = getGyroValFromBuffer(0);
            rawGyro[1] = getGyroValFromBuffer(1);
            rawGyro[2] = getGyroValFromBuffer(2);
        }
};
