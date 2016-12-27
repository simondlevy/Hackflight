/*
   board_rx_dsm.cpp : implementation of board-specific routines for Spektrum DSM receivers

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
#include <drivers/spektrum.h>

#include <math.h>

#include "board.hpp"

uint16_t Board::rcReadSerial(uint8_t chan)
{
    static uint8_t chanmap[5] = {1, 2, 3, 0, 4};
    return chan > 4 ? 0 : spektrumReadRawRC(chanmap[chan]);
}

bool Board::rcUseSerial(void)
{
    spektrumInit(USART2, SERIALRX_SPEKTRUM1024);

    return true;
}

uint16_t Board::rcReadPWM(uint8_t chan)
{
    (void)chan; // avoid compiler warning about unused variable
    return 0;
}

bool Board::rcSerialReady(void)
{
    return spektrumFrameComplete();
}


#ifdef __arm__
} // extern "C"
#endif
