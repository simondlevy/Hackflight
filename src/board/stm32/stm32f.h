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

#include "board/stm32.h"
#include "task/accelerometer.h"
#include "imu.h"

class Stm32FBoard : public Stm32Board {

    private:

        SPIClass m_spi;

        uint8_t m_csPin;

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        uint8_t m_buffer[15];

        void spiInit(        
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin)
        {
            m_spi.setMOSI(mosiPin);
            m_spi.setMISO(misoPin);
            m_spi.setSCLK(sclkPin);

            m_csPin = csPin;
        }
 
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

        void imuBegin(void)
        {
            switch (m_imu->type) {

                default:
                    break;
            }
        }

    protected:

        virtual void prioritizeExtraTasks(
                Task::prioritizer_t & prioritizer,
                const uint32_t usec) override
        {
            m_accelerometerTask.prioritize(usec, prioritizer);
            m_skyrangerTask.prioritize(usec, prioritizer);
        }

        virtual bool gyroIsReady(void) 
        {
            return true;
        }

        virtual void getRawGyro(int16_t rawGyro[3])
        {
            return m_imu->getRawGyro(rawGyro);
        }

    public:

        Stm32FBoard(
                Receiver & receiver,
                Imu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32Board(receiver, imu, pids, mixer, esc, ledPin)
        {
        }

        void begin(void)
        {
            Board::begin();

            imuBegin();

            m_accelerometerTask.begin(m_imu);
        }

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                m_skyrangerTask.parse(serial.read());
            }
        }

}; // class Stm32FBoard
