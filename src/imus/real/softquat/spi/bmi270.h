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

#include "imus/real/softquat/spi.h"

#include <SPI.h>

class Bmi270 : public SpiImu {

    public:

        uint8_t id;

    private:

        // Registers
        static const uint8_t REG_CHIP_ID = 0x00;

    protected:

        virtual void begin(void) override 
        {
            m_spi->begin();

            id = readRegister(REG_CHIP_ID);
        }

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            // Simulates rocking in the X (phi) axis

            (void)time;

            static float phi;
            static int8_t dir = +1;

            phi += .01 * dir;

            if (phi >= 1.0) {
                dir = -1;
            }

            if (phi <= -1.0) {
                dir = +1;
            }

            return Axes(phi, 0.1, 0.1);
        }

        virtual uint32_t getGyroInterruptCount(void) override
        {
            static uint32_t _count;
            static uint32_t _tprev;

            // Simulate 8kHz interrupts
            uint32_t time = micros();
            if (time - _tprev > 125) {
                _count++;
                _tprev = time;
            }

            return _count;
        }

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles) override
        {
            (void)nextTargetCycles;
            (void)desiredPeriodCycles;
            return 0;
        }

        virtual bool gyroIsCalibrating(void) override
        {
            return false;
        }

        virtual bool gyroIsReady(void) override
        {
            return false;
        }

        virtual auto readGyroDps(const align_fun align) -> Axes  override
        {
            (void)align;
            return Axes(0, 0, 0);
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            (void)k;
            return 0;
        }

    public:

        Bmi270(SPIClass & spi, const uint8_t csPin)
            : SpiImu(spi, csPin, 2000 / 32768.)
        {
        }

}; // class Bmi270
