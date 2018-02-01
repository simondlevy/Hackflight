/*
   cppm.hpp : Receiver subclass for CPPM receivers like FrSky MicroRX

   Uses moving average of channel values to remove noise

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.h

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "receiver.hpp"

namespace hf {

    class CPPM_Receiver: public Receiver {

        private: 

            int32_t ppmAverageIndex;  

            void init(void)
            {
                Receiver::init();

                ppmAverageIndex = 0;
            }

            void readRawvals(void)
            {
                float averageRaw[5][4];

                for (uint8_t chan = 0; chan < 5; chan++) {
                    averageRaw[chan][ppmAverageIndex % 4] = readChannel(chan);
                    rawvals[chan] = 0;
                    for (uint8_t i = 0; i < 4; i++)
                        rawvals[chan] += averageRaw[chan][i];
                    rawvals[chan] /= 4;
                }
                ppmAverageIndex++;
            }
    };
}
