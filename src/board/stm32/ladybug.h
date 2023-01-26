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

#include <Wire.h>

#include <USFS.h>

#include "board/stm32.h"
#include "esc/brushed.h"
#include "imu/ladybug.h"

class LadybugBoard : public Stm32Board {

    private:

        static const uint8_t IMU_INTERRUPT_PIN = 0x0C;

        std::vector<uint8_t> motorPins = {0x0D, 0x10, 0x03, 0x0B};

        LadybugImu m_imu; 

        BrushedEsc esc = BrushedEsc(motorPins);

    public:

        static const uint8_t LED_PIN = 0x12;

        LadybugBoard(Receiver & rx, std::vector<PidController *> & pids, Mixer & mixer)
            : Stm32Board(rx, &m_imu, pids, mixer, esc, -LED_PIN) // note inverted LED pin
        {
        }

        void begin(void (*isr)(void))
        {
            Serial.begin(115200);

            Wire.begin();
            Wire.setClock(400000); 
            delay(100);

            Board::setInterrupt(IMU_INTERRUPT_PIN, isr, RISING);  

            Board::begin();
        }

}; // class LadybugBoard
