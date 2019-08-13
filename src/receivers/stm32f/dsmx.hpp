/*
   Support for Spektrum DSMX receivers on STM32Fx boards

   Copyright (c) 2019 Simon D. Levy

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

#include <receiver.hpp>
#include <hackflight.hpp>
#include <DSMRX.h>

// Cleanflight drivers
extern "C" {
#include "drivers/time.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"
}

static DSM2048 * _rx;

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

static void serial_event(uint16_t value, void * data)
{
    (void)data;

    _dsmValue = (uint8_t)value;
    _dsmAvailable = 1;

    _rx->handleSerialEvent(micros());
}

class DSMX_Receiver : public hf::Receiver {

    public:

        DSMX_Receiver(UARTDevice_e uartDevice, const uint8_t channelMap[6]) 
            : Receiver(channelMap) 
        {       
            _uartDevice = uartDevice;  
        }


        virtual void begin(void) override
        {
            // Set up UART
            uartPinConfigure(serialPinConfig());

            // Open serial connection to receiver
            uartOpen(_uartDevice, serial_event, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);

            // Create a DSM2048 object to handle serial interrupts
            _rx = new DSM2048();
        }


        //protected:
    public:

        virtual bool gotNewFrame(void) override
        {
            return _rx->gotNewFrame();
        }

        void virtual readRawvals(void) override
        {
            _rx->getChannelValuesNormalized(rawvals, MAXCHAN);
        } 

    private:

        UARTDevice_e _uartDevice;

}; // DSMX_Receiver
