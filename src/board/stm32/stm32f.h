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
#include "imu/softquat/invensense.h"

class Stm32FBoard : public Stm32Board {

    private:

        static const uint8_t IMU_MOSI_PIN = PA7;
        static const uint8_t IMU_MISO_PIN = PA6;
        static const uint8_t IMU_SCLK_PIN = PA5;

        SPIClass m_spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

        InvenSenseImu * m_invenSenseImu;

        uint32_t m_initialSpiFreq; 
        uint32_t m_maxSpiFreq;

        // Enough room for seven two-byte integers (gyro XYZ, temperature,
        // accel XYZ) plus one byte for SPI transfer
        uint8_t m_buffer[15];

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_invenSenseImu->csPin, LOW);
            m_spi.transfer(reg);
            m_spi.transfer(val);
            digitalWrite(m_invenSenseImu->csPin, HIGH);
        }

        void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_invenSenseImu->csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi.transfer(buffer, count+1);
            digitalWrite(m_invenSenseImu->csPin, HIGH);
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
            readRegisters(
                    m_invenSenseImu->dataRegister,
                    m_invenSenseImu->buffer,
                    InvenSenseImu::BUFFER_SIZE);

            m_invenSenseImu->bufferToRawGyro(rawGyro);
        }

        Stm32FBoard(
                Receiver & receiver,
                InvenSenseImu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin)
            : Stm32Board(receiver, &imu, pids, mixer, esc, ledPin)
        {
            m_invenSenseImu = &imu;

            m_initialSpiFreq = imu.initialSpiFreq;
            m_maxSpiFreq = imu.maxSpiFreq;
        }

    public:

        void begin(void)
        {
            Board::begin();

            // Support MockImu
            if (m_invenSenseImu->csPin != 0) {

                m_spi.begin();

                pinMode(m_invenSenseImu->csPin, OUTPUT);

                const uint32_t clockSpeed = getClockSpeed();

                m_spi.setClockDivider(
                        InvenSenseImu::calculateSpiDivisor(clockSpeed, m_initialSpiFreq));

                std::vector<InvenSenseImu::registerSetting_t> registerSettings;

                m_invenSenseImu->getRegisterSettings(registerSettings);

                for (auto r : registerSettings) {
                    writeRegister(r.address, r.value);
                    delay(100); // arbitrary; should be long enough for any delays
                }

                // ICM42688 requires register reset for interrupt
                const uint8_t ICM426XX_RA_INT_CONFIG1 = 0x64;
                const uint8_t ICM426XX_INT_ASYNC_RESET_BIT = 4;
                const uint8_t ICM426XX_INT_TDEASSERT_DISABLE_BIT  = 5;
                const uint8_t ICM426XX_INT_TPULSE_DURATION_BIT = 6;
                const uint8_t ICM426XX_INT_TPULSE_DURATION_8 = (1 << ICM426XX_INT_TPULSE_DURATION_BIT);
                const uint8_t ICM426XX_INT_TDEASSERT_DISABLED  = (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT);
                uint8_t intConfig1Value = readRegister(ICM426XX_RA_INT_CONFIG1);
                intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
                intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);
                writeRegister(ICM426XX_RA_INT_CONFIG1, intConfig1Value);
                delay(100);

                m_spi.setClockDivider(
                        InvenSenseImu::calculateSpiDivisor(clockSpeed, m_maxSpiFreq));
            }

            m_accelerometerTask.begin(m_imu);
        }

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                m_skyrangerTask.parse(serial.read());
            }
        }

}; // class Stm32FBoard
