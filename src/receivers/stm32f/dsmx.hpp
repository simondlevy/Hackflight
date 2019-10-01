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

static void serial_event(uint16_t value, void * data)
{

    DSM2048 * rx = (DSM2048 *)data;

    rx->handleSerialEvent((uint8_t)value, micros());
}

class DSMX_Receiver : public hf::Receiver {

    public:

        DSMX_Receiver(UARTDevice_e uartDevice, const uint8_t channelMap[6], const float demandScale) 
            : Receiver(channelMap, demandScale) 
        {       
            _uartDevice = uartDevice;  
        }


        virtual void begin(void) override
        {
            // Set up UART
            uartPinConfigure(serialPinConfig());

            // Create a DSM2048 object to handle serial interrupts
            _rx = new DSM2048();

            // Open serial connection to receiver
            uartOpen(_uartDevice, serial_event, _rx,  115200, MODE_RX, SERIAL_NOT_INVERTED);
        }

    protected:

        virtual bool gotNewFrame(void) override
        {
            return _rx->gotNewFrame();
        }

        void virtual readRawvals(void) override
        {
            _rx->getChannelValuesNormalized(rawvals, MAXCHAN);
        } 

    private:

        DSM2048 * _rx = NULL;

        UARTDevice_e _uartDevice;

}; // DSMX_Receiver
