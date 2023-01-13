/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

   Copyright (c) 2022 Simon D. Levy

   MIT License
 */

#include <SPI.h>

#include <stdint.h>

#include "imu/real/softquat.h"

class Bmi270 : public SoftQuatImu {

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
        static const uint8_t REG_CHIP_ID = 0x00;

        SPIClass m_spi;

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_csPin, LOW);
            m_spi.transfer(reg);
            m_spi.transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi.transfer(buffer, count+1);
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
            m_spi.setClockDivider(divider);
        }

        uint8_t m_mosiPin;
        uint8_t m_misoPin;
        uint8_t m_sclkPin;
        uint8_t m_csPin;

        uint16_t calculateSpiDivisor(const uint32_t freq)
        {
            uint32_t clk = m_board->getClockSpeed() / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

        virtual bool gyroIsReady(void) override
        {

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(void) override
        {
            m_spi.setMOSI(m_mosiPin);
            m_spi.setMISO(m_misoPin);
            m_spi.setSCLK(m_sclkPin);

            m_spi.begin();
            m_spi.setBitOrder(MSBFIRST);
            // m_spi.setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));
            m_spi.setDataMode(SPI_MODE3);
            pinMode(m_csPin, OUTPUT);
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            return 0;
        }

        static uint16_t gyroScaleToInt(const gyroScale_e gyroScale)
        {
            return
                gyroScale == GYRO_250DPS ?  250 : 
                gyroScale == GYRO_500DPS ?  500 : 
                gyroScale == GYRO_1000DPS ?  1000 : 
                2000;
        }

    public:

        Bmi270(
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const uint8_t sampleRateDivisor = 19,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : SoftQuatImu(rotateFun, gyroScaleToInt(gyroScale) / 32768.)
        {
            m_mosiPin = mosiPin;
            m_misoPin = misoPin;
            m_sclkPin = sclkPin;
            m_csPin = csPin;
            //m_sampleRateDivisor = sampleRateDivisor;
            //m_gyroScale = gyroScale;
            //m_accelScale = accelScale;
        }

}; // class Bmi270
