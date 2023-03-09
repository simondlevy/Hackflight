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

#include "board.h"
#include "tasks/accelerometer.h"
#include "imus/softquat.h"

class Stm32FBoard : public Stm32Board {

    protected:

        virtual void prioritizeExtraTasks(
                Task::prioritizer_t & prioritizer,
                const uint32_t usec) override
        {
            m_logic.prioritizeExtraTasks(prioritizer, usec);
        }

        Stm32FBoard(const uint8_t ledPin)
            : Stm32Board(ledPin)
        {
        }

    public:

        static const uint8_t MOSI_PIN = PA7;
        static const uint8_t MISO_PIN = PA6;
        static const uint8_t SCLK_PIN = PA5;

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                m_logic.skyrangerParseData(serial.read());
            }
        }

}; // class Stm32FBoard
