/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

   Copyright (c) 2022 Simon D. Levy

   MIT License
*/

#include "imu/real/softquat/mpu6x00.h"

#include <SPI.h>

class ArduinoMpu6x00 : public Mpu6x00 {

    private:

        SPIClass * m_spi;

    protected:

        virtual void writeRegister(const uint8_t reg, const uint8_t val) override
        {
            digitalWrite(m_csPin, LOW);
            m_spi->transfer(reg);
            m_spi->transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        virtual void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count) override
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi->transfer(buffer, count);
            digitalWrite(m_csPin, HIGH);
        }

        virtual uint8_t readRegister(const uint8_t addr) override
        {
            uint8_t buffer[2] = {};
            readRegisters(addr, buffer, 2);
            return buffer[1];
        }

        virtual void setClockDivider(uint32_t divider) override
        {
            m_spi->setClockDivider(divider);
        }

    public:

        ArduinoMpu6x00(
                SPIClass & spi,
                const rotateFun_t rotateFun,
                const uint8_t csPin,
                const uint8_t sampleRateDivisor = 19,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : Mpu6x00(rotateFun, csPin, sampleRateDivisor, gyroScale, accelScale)
 
        {
            m_spi = &spi;
        }

        virtual void begin(void) override
        {
            m_spi->begin();
            m_spi->setBitOrder(MSBFIRST);
            m_spi->setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));
            m_spi->setDataMode(SPI_MODE3);
            pinMode(m_csPin, OUTPUT);

            Mpu6x00::begin();
        }

}; // class ArduinoMpu6x00
