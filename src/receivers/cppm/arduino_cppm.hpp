/*
   arduino_cppm.hpp : CPPM receiver support for Arduino-based flight controllers

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

#pragma once

#include "receiver.hpp"
#include <CPPM.h>

namespace hf {

    class CPPM_Receiver : public Receiver {

        private:

            CPPM * rx;

            uint32_t ppmAverageIndex;  

        protected:

            void begin(void)
            {
                ppmAverageIndex = 0;
            }

            bool gotNewFrame(void)
            {
                return rx->gotNewFrame();
            }

            void readRawvals(void)
            {
                uint16_t rcData[8];

                rx->computeRC(rcData);

                float averageRaw[5][4];

                for (uint8_t chan = 0; chan < 6; chan++) {
                    averageRaw[chan][ppmAverageIndex % 4] = (rcData[chan] - 1000) / 500.f - 1;
                    rawvals[chan] = 0;
                    for (uint8_t i = 0; i < 4; i++)
                        rawvals[chan] += averageRaw[chan][i];
                    rawvals[chan] /= 4;
                }
                ppmAverageIndex++;
            }

            bool lostSignal(void)
            {
                return false;
            }

        public:

            CPPM_Receiver(uint8_t pin, const uint8_t channelMap[6]) : Receiver(channelMap) 
            { 
                rx = new CPPM(pin, 6);
            }

    }; // class CPPM_Receiver

} // namespace hf
