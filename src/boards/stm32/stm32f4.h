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

#include "boards/stm32.h"

#include <stm32f4xx.h>

class Stm32F4Board : public Stm32Board {

    public:

        Stm32F4Board(
                Receiver & receiver,
                Imu & imu,
                Imu::align_fun align,
                vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32Board(receiver, imu, align, pids, mixer, esc, ledPin)
        {
        }

}; // class Stm32F4Board
