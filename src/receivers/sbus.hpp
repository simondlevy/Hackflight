/*
   sbus.hpp : Futaba SBUS receiver support for Ladybug Flight Controller

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

#include "receiver.hpp"
#include <SBUS.h>

static SBUS rx(Serial1);

namespace hf {

    class SBUS_Receiver : public Receiver {

        void begin(void)
        {
            rx.begin();
        }

        bool useSerial(void)
        {
            return true;
        }

        float readChannel(uint8_t chan)
        {
            // grab all channels on first channel request
            if (chan == 0) {
                rx.readCal(channels, &failSafe, &lostFrames);
            }

            return channels[chan];
        }

        bool lostSignal(void)
        {
            return false;
        }

        private:
        float channels[16];
        uint8_t failSafe;
        uint16_t lostFrames = 0;

    }; // class DSMX_Receiver

} // namespace
