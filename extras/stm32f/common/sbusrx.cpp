/*
   sbusrx.cpp Support for Futaba SBUS receivers on STM32Fx boards

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


#include "sbusrx.h"
#include <SBUSRX.h>
#include <debug.hpp>

static SBUSRX * _rx;

// Support for reading SBUSRX signals over UART
static uint8_t _sbusAvailable;
static uint8_t _sbusValue;

uint8_t sbusSerialAvailable(void)
{
    return _sbusAvailable;
}

uint8_t sbusSerialRead(void)
{
    _sbusAvailable--;
    return _sbusValue;
}

extern "C" {

#include "drivers/time.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"

static void serial_event(uint16_t value, void * data)
{
    (void)data;

    _sbusValue = (uint8_t)value;
    _sbusAvailable = 1;

    _rx->handleSerialEvent(micros());
}

SBUS_Receiver::SBUS_Receiver(UARTDevice_e uartDevice, const uint8_t channelMap[6]) : Receiver(channelMap) 
{       
    _uartDevice = uartDevice;  
}

void SBUS_Receiver::begin(void) 
{
    // Set up UART
    uartPinConfigure(serialPinConfig());

    // Open serial connection to receiver
    uartOpen(_uartDevice, serial_event, NULL, 100000, MODE_RX, 
            (portOptions_e)((uint8_t)SERIAL_STOPBITS_2|(uint8_t)SERIAL_PARITY_EVEN|(uint8_t)SERIAL_INVERTED));


    // Create a SBUSRX object to handle serial interrupts
    _rx = new SBUSRX();
}

bool SBUS_Receiver::gotNewFrame(void) 
{
    return _rx->gotNewFrame();
}

void SBUS_Receiver::readRawvals(void)
{
    uint8_t failsafe = 0;
    uint16_t lostFrames = 0;

    _rx->getChannelValuesNormalized(rawvals, &failsafe, &lostFrames);
}

} // extern "C"
