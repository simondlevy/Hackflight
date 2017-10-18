/*
   dsmx.hpp : Spektrum DSMX support for Ladybug Flight Controller

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
#include <SpektrumDSM.h>

static SpektrumDSM2048 rx;

namespace hf {

    class DSMX_Receiver : public Receiver {

        void begin(void)
        {
            rx.begin();
        }

        bool useSerial(void)
        {
            return true;
        }

        uint16_t readChannel(uint8_t chan)
        {
            // TAER
            uint8_t chanmap[5] = {0, 1, 2, 3, 4};
            return rx.getChannelValue(chanmap[chan]);
        }

        bool lostSignal(void)
        {
            // Perhaps we should tolerate a higher fade count?
            return rx.timedOut() || (rx.getFadeCount() > 0);
        }

    }; // class DSMX_Receiver

} // namespace
