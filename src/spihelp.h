/*
   SPI I/O methods

   Copyright (c) 2022 Simon D. Levy

   MIT License
*/

#include <SPI.h>

class SpiHelper {

    public:

        static void writeRegister(
                SPIClass * spi,
                const uint8_t cspin,
                const uint8_t reg,
                const uint8_t val)
        {
            digitalWrite(cspin, LOW);
            spi->transfer(reg);
            spi->transfer(val);
            digitalWrite(cspin, HIGH);
        }

        static void readRegisters(
                SPIClass * spi,
                const uint8_t cspin,
                const uint8_t addr,
                uint8_t * buffer,
                const uint8_t count)
        {
            digitalWrite(cspin, LOW);
            buffer[0] = addr | 0x80;
            spi->transfer(buffer, count);
            digitalWrite(cspin, HIGH);
        }

        static uint8_t readRegister(
                SPIClass * spi,
                const uint8_t cspin,
                const uint8_t addr)
        {
            uint8_t buffer[2] = {};
            readRegisters(spi, cspin, addr, buffer, 2);
            return buffer[1];
        }

}; // class SpiHelper
