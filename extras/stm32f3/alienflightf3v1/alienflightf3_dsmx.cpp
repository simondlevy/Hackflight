/*
   alienflightf3_dsmx.cpp Support for Spektrum DSMX receive on AlienflightF3 board

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

#include "alienflightf3_dsmx.h"
#include <SpektrumDSM.h>

SpektrumDSM2048 * rx;

// Support for reading DSMX signals over UART2
static uint8_t dsmAvailable;
static uint8_t dsmValue;

uint8_t dsmSerialAvailable(void)
{
    return dsmAvailable;
}

uint8_t dsmSerialRead(void)
{
    dsmAvailable--;
    return dsmValue;
}

extern "C" {

#include "platform.h"
#include "dma.h"
#include "serial.h"
#include "system.h"
#include "serial_uart.h"
#include "system.h"

static serialPort_t * serial2;

static void serial_event_2(uint16_t value)
{
    dsmValue = (uint8_t)value;
    dsmAvailable = 1;

    rx->handleSerialEvent(micros());
}

static void serial2Open(void)
{
    // Open connection to UART2
    serial2 = uartOpen(USART2, serial_event_2, 115200, MODE_RX, SERIAL_NOT_INVERTED);
}

} // extern "C"

DSMX_Receiver::DSMX_Receiver(const uint8_t channelMap[6], float trimRoll, float trimPitch, float trimYaw) : 
    Receiver(channelMap, trimRoll, trimPitch, trimYaw) 
{         

    // Open connection to UART2
    serial2Open();

    // Create a SpektrumDSM2048 object to handle serial interrupts
    rx = new SpektrumDSM2048();
}

bool DSMX_Receiver::gotNewFrame(void) 
{
    return rx->gotNewFrame();
}

void DSMX_Receiver::readRawvals(void)
{
    rx->getChannelValuesNormalized(rawvals, CHANNELS);
}
