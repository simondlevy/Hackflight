/*
   Spektrum DSMX support for Arduino flight controllers using Serial1

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

#pragma once

#include "receivers/arduino/dsmx.hpp"
#include <DSMRX.h>

static hf::DSMX_Receiver * _dsmx_rx;

void serialEvent1(void)
{
    while (Serial1.available()) {

        _dsmx_rx->handleSerialEvent(Serial1.read(), micros());
    }
}

namespace hf {

    class DSMX_Receiver_Serial1 : public DSMX_Receiver {

        protected:

            void begin(void) override 
            {
                Receiver::begin();

                Serial1.begin(115200);
            }

        public:

            DSMX_Receiver_Serial1(const uint8_t channelMap[6])
                :  DSMX_Receiver(channelMap) 
            { 
                _dsmx_rx = this;
            }

    }; // class DSMX_Receiver_Serial1

} // namespace hf
