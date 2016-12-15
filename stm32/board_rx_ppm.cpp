/*
   board_rx_ppm.cpp : implementation of board-specific routines for PPM receivers

   This implemenation is for STM32F103 boards (Naze32, Flip32, etc.)

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

#ifdef __arm__
extern "C" {
#endif

#include <breezystm32.h>
#include <drivers/mpu6050.h>
#include <drivers/ms5611.h>

#include <math.h>

#include "board.hpp"
#include "motorpwm.hpp"

uint16_t Board::rcReadSerial(uint8_t chan)
{
    (void)chan;
    return 0;
}

bool Board::rcUseSerial(void)
{

    return false;
}

uint16_t Board::rcReadPWM(uint8_t chan)
{
    return pwmRead(chan);
}

bool Board::rcSerialReady(void)
{
    return false;
}

#ifdef __arm__
} // extern "C"
#endif
