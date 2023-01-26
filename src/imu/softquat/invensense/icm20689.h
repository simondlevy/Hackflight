/*
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

#include <SPI.h>
#include <Arduino.h>

class ICM20689 {

    private:

        static const uint8_t ACCEL_OUT = 0x3B;
        static const uint8_t ACCEL_CONFIG = 0x1C;

        static const uint8_t ACCEL_FS_SEL_2G = 0x00;
        static const uint8_t ACCEL_FS_SEL_4G = 0x08;
        static const uint8_t ACCEL_FS_SEL_8G = 0x10;
        static const uint8_t ACCEL_FS_SEL_16G = 0x18;

        static const uint8_t GYRO_CONFIG = 0x1B;
        static const uint8_t GYRO_FS_SEL_250DPS = 0x00;
        static const uint8_t GYRO_FS_SEL_500DPS = 0x08;
        static const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
        static const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

        static const uint8_t ACCEL_CONFIG2 = 0x1D;
        static const uint8_t ACCEL_DLPF_218HZ = 0x01;
        static const uint8_t ACCEL_DLPF_99HZ = 0x02;
        static const uint8_t ACCEL_DLPF_45HZ = 0x03;
        static const uint8_t ACCEL_DLPF_21HZ = 0x04;
        static const uint8_t ACCEL_DLPF_10HZ = 0x05;
        static const uint8_t ACCEL_DLPF_5HZ = 0x06;
        static const uint8_t ACCEL_DLPF_420HZ = 0x07;
        static const uint8_t ACCEL_DLPF_1046HZ = 0x08;

        static const uint8_t CONFIG = 0x1A;

        static const uint8_t GYRO_DLPF_250HZ = 0x00;
        static const uint8_t GYRO_DLPF_176HZ = 0x01;
        static const uint8_t GYRO_DLPF_92HZ = 0x02;
        static const uint8_t GYRO_DLPF_41HZ = 0x03;
        static const uint8_t GYRO_DLPF_20HZ = 0x04;
        static const uint8_t GYRO_DLPF_10HZ = 0x05;
        static const uint8_t GYRO_DLPF_5HZ = 0x06;

        static const uint8_t SMPLRT_DIV = 0x19;

        static const uint8_t INT_PIN_CFG = 0x37;
        static const uint8_t INT_ENABLE = 0x38;
        static const uint8_t INT_PULSE_50US = 0x00;
        static const uint8_t INT_RAW_RDY_EN = 0x01;
        static const uint8_t INT_STATUS = 0x3A;

        static const uint8_t PWR_MGMT_1 = 0x6B;
        static const uint8_t PWR_RESET = 0x80;

        static const uint8_t CLOCK_SEL_PLL = 0x01;

        static const uint8_t PWR_MGMT_2 = 0x6C;

        static const uint8_t SEN_ENABLE = 0x00;

        SPIClass * _spi;
        uint8_t _csPin;

        const uint32_t SPI_INIT_CLK_HZ = 1000000; 
        const uint32_t SPI_CLK_HZ      = 8000000; 
        
        uint8_t _buffer[15] = {};
        
        void writeRegister(const uint8_t addr, const uint8_t val) 
        {
            _spi->beginTransaction(SPISettings(SPI_INIT_CLK_HZ, MSBFIRST, SPI_MODE3)); 

            digitalWrite(_csPin,LOW); 
            _spi->transfer(addr); 
            _spi->transfer(val); 
            digitalWrite(_csPin,HIGH); 

            _spi->endTransaction(); 

            delay(10);

            readRegisters(addr, 1, _buffer, SPI_INIT_CLK_HZ);
        }

        void readRegisters(
                const uint8_t addr,
                const uint8_t count,
                uint8_t * buffer,
                const uint32_t spiClkHz) {

            _spi->beginTransaction(SPISettings(spiClkHz, MSBFIRST, SPI_MODE3));

            digitalWrite(_csPin,LOW); 

            buffer[0] = addr | 0x80;
            _spi->transfer(buffer, count+1);

            digitalWrite(_csPin,HIGH); 

            _spi->endTransaction(); 
        }

    public:

        int16_t accelCounts[3] = {};
        int16_t gyroCounts[3] = {};

        ICM20689(SPIClass &bus,uint8_t csPin) 
        {
            _spi = &bus; 
            _csPin = csPin; 
        }

        void begin() 
        {
            pinMode(_csPin,OUTPUT);

            digitalWrite(_csPin,HIGH);

            _spi->begin();

            writeRegister(PWR_MGMT_1,CLOCK_SEL_PLL);

            writeRegister(PWR_MGMT_1,PWR_RESET);

            delay(1);

            writeRegister(PWR_MGMT_1,CLOCK_SEL_PLL);

            writeRegister(PWR_MGMT_2,SEN_ENABLE);

            writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G);

            writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS);

            writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_1046HZ);

            writeRegister(CONFIG,GYRO_DLPF_250HZ);

            writeRegister(SMPLRT_DIV,0x00);

            writeRegister(INT_PIN_CFG,INT_PULSE_50US);

            writeRegister(INT_ENABLE,INT_RAW_RDY_EN);
        }

        void readSensor() {

            readRegisters(ACCEL_OUT, 15, _buffer, SPI_CLK_HZ);

            accelCounts[0] = (((int16_t)_buffer[1]) << 8) | _buffer[2];
            accelCounts[1] = (((int16_t)_buffer[3]) << 8) | _buffer[4];
            accelCounts[2] = (((int16_t)_buffer[5]) << 8) | _buffer[6];

            gyroCounts[0] = (((int16_t)_buffer[9]) << 8) | _buffer[10];
            gyroCounts[1] = (((int16_t)_buffer[11]) << 8) | _buffer[12];
            gyroCounts[2] = (((int16_t)_buffer[13]) << 8) | _buffer[14];
        }
}; 
