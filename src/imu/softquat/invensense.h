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

        } gyroScale_e;

        typedef enum {

            ACCEL_2G,
            ACCEL_4G,  
            ACCEL_8G,  
            ACCEL_16G

        } accelScale_e;

    private:

        // Shared with Stm32FBoard
        uint8_t dataRegister;
        uint32_t initialSpiFreq;
        uint32_t maxSpiFreq;

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

        static uint16_t calculateSpiDivisor(const uint32_t clockSpeed, const uint32_t freq)
        {
            uint32_t clk = clockSpeed / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

    protected:

        typedef struct {

            uint8_t address;
            uint8_t value;

        } registerSetting_t;

        virtual void getRegisterSettings(std::vector<registerSetting_t> & settings) = 0;
 
        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        static const uint8_t BUFFER_SIZE = 15;

        uint8_t buffer[BUFFER_SIZE];

        int16_t getShortFromBuffer(const uint8_t offset, const uint8_t index)
        {
            const uint8_t k = 2 * (offset + index) + 1;
            return (int16_t)(buffer[k] << 8 | buffer[k+1]);
        }

        int16_t getGyroValFromBuffer(uint8_t k)
        {
            return getShortFromBuffer(4, k);
        }

        void bufferToRawGyro(int16_t rawGyro[3])
        {
            rawGyro[0] = getGyroValFromBuffer(0);
            rawGyro[1] = getGyroValFromBuffer(1);
            rawGyro[2] = getGyroValFromBuffer(2);
        }

        InvenSenseImu(
                const uint32_t initialSpiFreq,
                const uint32_t maxSpiFreq,
                const uint8_t dataRegister,
                const rotateFun_t rotateFun,
                const gyroScale_e gyroScale,
                const accelScale_e accelScale)
            : SoftQuatImu(
                    rotateFun,
                    gyroScaleToInt(gyroScale),
                    accelScaleToInt(accelScale))
        {
            this->dataRegister = dataRegister;

            this->initialSpiFreq = initialSpiFreq;
            this->maxSpiFreq = maxSpiFreq;
        }

        void begin(const uint32_t mcuClockSpeed)
        {
            SoftQuatImu::begin(mcuClockSpeed);
        }

        void getRawGyro(int16_t rawGyro[3])
        {
            rawGyro[0] = getShortFromBuffer(4, 0);
            rawGyro[1] = getShortFromBuffer(4, 1);
            rawGyro[2] = getShortFromBuffer(4, 2);
        }

};
