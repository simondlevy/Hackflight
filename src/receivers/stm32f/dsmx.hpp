/*
   dsmx.h Support for Spektrum DSMX receivers on STM32Fx boards

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

#include <receiver.hpp>
#include <hackflight.hpp>

extern "C" {

#include "io/serial.h"
#include "drivers/serial_uart.h"

class DSMX_Receiver : public hf::Receiver {

    public:

        DSMX_Receiver(UARTDevice_e uartDevice, const uint8_t channelMap[6]);

        virtual void begin(void) override;

    //protected:
    public:

        virtual bool gotNewFrame(void) override;

        virtual void readRawvals(void) override;

    private:

        UARTDevice_e _uartDevice;

}; // DSMX_Receiver

} // extern "C"
