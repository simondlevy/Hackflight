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

BrushedMotor * motor;

void F3Board::writeMotor(uint8_t index, float value)
{
    (void)index;

    motor->writeMicroseconds((uint16_t)(1000*(value+1)));

    hf::Debug::printf("%d\n", (int)(100*value));
}

void F3Board::motorInit(void)
{
    // Valid pins for ALIENFLIGHTF3 are 0, 8, 14, 15
    motor = new BrushedMotor(14);
}

void F3Board::adjustImu(float & a1, float & a2, float & g1, float & g2)
{
    a1 =  _ay;
    a2 =  _ax,
    g1 = -_gy;
    g2 = -_gx;
}


