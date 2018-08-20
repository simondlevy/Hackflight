/*
   alienflightf3.cpp Board class implementation for F3Board

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <f3_board.h>
#include <motor.h>
#include <debug.hpp>

uint8_t F3Board::getI2cBus(void)
{
    return 2;
}

static BrushedMotor * motors[4];

void F3Board::writeMotor(uint8_t index, float value)
{
    motors[index]->writeMicroseconds((uint16_t)(1000*(value+1)));

    if (index == 0) {
        extern uint32_t motorval;
        hf::Debug::printf("%d\n", motorval);
    }
}

void F3Board::motorInit(void)
{
    // Valid pins for ALIENFLIGHTF3 are 0, 8, 14, 15
    motors[0] = new BrushedMotor(15);
    motors[1] = new BrushedMotor(14);
    motors[2] = new BrushedMotor(8);
    motors[3] = new BrushedMotor(0);
}

extern "C" {

#include "platform.h"
#include "serial.h"

    USART_TypeDef * getDsmUart()
    {
        return USART2;
    }

} // extern "C"
