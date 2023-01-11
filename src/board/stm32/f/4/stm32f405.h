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

#include "board/stm32/f/stm32f4.h"

#include <stm32f4xx.h>

class Stm32F405Board : public Stm32F4Board {

    protected:

        virtual void initPortsAndMotors(const vector<uint8_t> * motorPins) override
        {
            initStream1(1);
            initStream2(0);

            initMotor(motorPins, 0, 0); 
            initMotor(motorPins, 1, 0);
            initMotor(motorPins, 2, 1);
            initMotor(motorPins, 3, 1);
        }        

    public:

        Stm32F405Board(
                Receiver & receiver,
                Imu & imu,
                vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32F4Board(2, receiver, imu, pids, mixer, esc, ledPin)
        {
        }
};
