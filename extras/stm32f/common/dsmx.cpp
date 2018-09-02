/*
   dsmx.cpp Support for Spektrum DSMX receive on STM32Fx boards

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

#include "dsmx.h"
#include <SpektrumDSM.h>
#include <debug.hpp>

static SpektrumDSM2048 * _rx;

// Support for reading DSMX signals over UART
static uint8_t _dsmAvailable;
static uint8_t _dsmValue;

uint8_t dsmSerialAvailable(void)
{
    return _dsmAvailable;
}

uint8_t dsmSerialRead(void)
{
    _dsmAvailable--;
    return _dsmValue;
}

extern "C" {

#include "drivers/time.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"

static void serial_event(uint16_t value, void * data)
{
    (void)data;

    _dsmValue = (uint8_t)value;
    _dsmAvailable = 1;

    _rx->handleSerialEvent(micros());
}

DSMX_Receiver::DSMX_Receiver(UARTDevice_e uartDevice,
        const uint8_t channelMap[6], float trimRoll, float trimPitch, float trimYaw) : 
    Receiver(channelMap, trimRoll, trimPitch, trimYaw) 
{       
    _uartDevice = uartDevice;  
}

void DSMX_Receiver::begin(void) 
{
    // Set up UART
    uartPinConfigure(serialPinConfig());

    // Open serial connection to receiver
    uartOpen(_uartDevice, serial_event, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);

    // Create a SpektrumDSM2048 object to handle serial interrupts
    _rx = new SpektrumDSM2048();
}

bool DSMX_Receiver::gotNewFrame(void) 
{
    return _rx->gotNewFrame();
}

void DSMX_Receiver::readRawvals(void)
{
    _rx->getChannelValuesNormalized(rawvals, CHANNELS);
}

} // extern "C"
