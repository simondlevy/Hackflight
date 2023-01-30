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
#include "imu/softquat/mpu6000.h"

class Stm32FBoard : public Stm32Board {

    private:

        Mpu6000 * m_invenSenseImu;

    protected:

        virtual void prioritizeExtraTasks(
                Task::prioritizer_t & prioritizer,
                const uint32_t usec) override
        {
            m_accelerometerTask.prioritize(usec, prioritizer);
            m_skyrangerTask.prioritize(usec, prioritizer);
        }

        Stm32FBoard(
                Mpu6000 & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin)
            : Stm32Board(&imu, pids, mixer, esc, ledPin)
        {
            m_invenSenseImu = &imu;
        }

    public:

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                m_skyrangerTask.parse(serial.read());
            }
        }

}; // class Stm32FBoard
