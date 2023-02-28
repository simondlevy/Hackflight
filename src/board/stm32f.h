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
#include "task/accelerometer.h"
#include "imu/softquat.h"

class Stm32FBoard : public Stm32Board {

    protected:

        Stm32FBoard(Esc & esc, const uint8_t ledPin)
            : Stm32Board(esc, ledPin)
        {
        }

    public:

        void handleSkyrangerEvent(Core & core, HardwareSerial & serial)
        {
            while (serial.available()) {
                core.parseSkyrangerData(serial.read());
            }
        }

}; // class Stm32FBoard
