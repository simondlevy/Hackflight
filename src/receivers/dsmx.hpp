/*
   dsmx.hpp : Spektrum DSMX support for Arduino flight controllers

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

#pragma once

#include "debug.hpp"
#include "receiver.hpp"
#include <DSMRX.h>

static DSM2048 rx;

// Support different UARTs

static HardwareSerial * _hardwareSerial;

void serialEvent1(void)
{
    rx.handleSerialEvent(micros());
}

void serialEvent2(void)
{
    rx.handleSerialEvent(micros());
}

void serialEvent3(void)
{
    rx.handleSerialEvent(micros());
}

uint8_t dsmSerialAvailable(void)
{
    return _hardwareSerial->available();
}

uint8_t dsmSerialRead(void)
{
    return _hardwareSerial->read();
}


namespace hf {

    class DSMX_Receiver : public Receiver {

         protected:

            void begin(void)
            {
                _hardwareSerial->begin(115200);
            }

            bool gotNewFrame(void)
            {
                return rx.gotNewFrame();
            }

            void readRawvals(void)
            {
                rx.getChannelValuesNormalized(rawvals, CHANNELS);
            }

            bool lostSignal(void)
            {
                return rx.timedOut(micros());
            }

        public:

            DSMX_Receiver(const uint8_t channelMap[6], HardwareSerial * hardwareSerial=&Serial1) 
                :  Receiver(channelMap) 
            { 
                _hardwareSerial = hardwareSerial;
            }

    }; // class DSMX_Receiver

} // namespace hf
