/*
   Header-only class definition for MPU6000 sensor using SPI bus

   Copyright (c) 2022 Simon D. Levy

   MIT License
*/

#include "imus/real/softquat.h"

#include <SPI.h>

class SpiImu : public SoftQuatImu {

    protected:

        SPIClass * m_spi;

        uint8_t m_csPin;

        SpiImu(SPIClass & spi, const uint8_t csPin, const float gyroScale)
            : SoftQuatImu(gyroScale)
        {
            m_spi = &spi;
            m_csPin = csPin;
        }

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_csPin, LOW);
            m_spi->transfer(reg);
            m_spi->transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        void readRegisters(const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi->transfer(buffer, count);
            digitalWrite(m_csPin, HIGH);
        }

        uint8_t readRegister(const uint8_t addr)
        {
            uint8_t buffer[2] = {};
            readRegisters(addr, buffer, 2);
            return buffer[1];
        }

}; // class SpiImu
