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

#include <SPI.h>

#include <string.h>

#include "core/axes.h"
#include "core/clock.h"
#include "core/filters/pt2.h"
#include "core/vstate.h"
#include "imu/softquat.h"

class InvenSenseImu : public SoftQuatImu {

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

        uint8_t m_misoPin;
        uint8_t m_mosiPin;
        uint8_t m_sclkPin;
        uint8_t m_csPin;

        void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi.transfer(buffer, count+1);
            digitalWrite(m_csPin, HIGH);
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


    protected:

        SPIClass m_spi;

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        uint8_t m_buffer[15];

        int16_t getShortFromBuffer(const uint8_t offset, const uint8_t index)
        {
            const uint8_t k = 2 * (offset + index) + 1;
            return (int16_t)(m_buffer[k] << 8 | m_buffer[k+1]);
        }

        InvenSenseImu(
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const gyroScale_e gyroScale,
                const accelScale_e accelScale)
            : SoftQuatImu(
                    rotateFun, gyroScaleToInt(gyroScale), accelScaleToInt(accelScale))
        {
            m_mosiPin = mosiPin;
            m_misoPin = misoPin;
            m_sclkPin = sclkPin;
            m_csPin = csPin;
        }

        void begin(uint32_t clockSpeed)
        {
            m_spi.setMOSI(m_mosiPin);
            m_spi.setMISO(m_misoPin);
            m_spi.setSCLK(m_sclkPin);
            m_spi.begin();

            pinMode(m_csPin, OUTPUT);

            SoftQuatImu::begin(clockSpeed);
        }

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_csPin, LOW);
            m_spi.transfer(reg);
            m_spi.transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        void readRegisters(const uint8_t addr)
        {
            readRegisters(addr, m_buffer, 14);
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

};
