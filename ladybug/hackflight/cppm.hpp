/*
   cppm.hpp : CPPM receiver support for Ladybug Flight Controller

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

#include <BreezyCPPM.h>

// Interrupt on pin 0, using 5 channels
static BreezyCPPM rx(0, 5);

namespace hf {

    void Ladybug::rcInit(void)
    {
        rx.begin();
    }

    bool Ladybug::rcUseSerial(void)
    {
        return false;
    }

    uint16_t Ladybug::rcReadChannel(uint8_t chan)
    {
        static uint16_t chanvals[5];

        // Only need to read channels once
        if (chan == 0) {
            rx.computeRC(chanvals);
        }

	// TAER
        return chanvals[chan];
    }

} // namespace
