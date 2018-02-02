/*
   arduino_cppm.hpp : CPPM receiver support for Arduino-based flight controllers

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
#include "cppm.hpp"

// Interrupt on pin 0, using 5 channels
static BreezyCPPM rx(0, 5);

namespace hf {

    class Arduino_CPPM_Receiver : public CPPM_Receiver {

        public:

            Arduino_CPPM_Receiver(float trimRoll=0, float trimPitch=0, float trimYaw=0) : CPPM_Receiver(trimRoll, trimPitch, trimYaw) { }

        protected:

            void begin(void)
            {
                rx.begin();
            }

            void readPulseVals(uint16_t pulsevals[8])
            {
                rx.computeRC(pulsevals);
            }

    }; // class Arduino_CPPM_Receiver

} // namespace hf
