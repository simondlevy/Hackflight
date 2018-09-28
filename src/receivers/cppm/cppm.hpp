/*
   cppm.hpp : Receiver subclass for CPPM receivers like FrSky MicroRX

   Uses moving average of channel values to remove noise

   Copyright (c) 2018 Simon D. Levy

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

        protected:

            CPPM_Receiver(const uint8_t channelMap[6]) :  Receiver(channelMap) 
            {
            }

            virtual void readPulseVals(uint16_t chanvals[8]) = 0;

        private: 

            int32_t ppmAverageIndex;  

            void init(void)
            {
                Receiver::init();

                ppmAverageIndex = 0;
            }

            void readRawvals(void)
            {
                uint16_t pulsevals[8];
                readPulseVals(pulsevals);

                float averageRaw[5][4];

                for (uint8_t chan = 0; chan < 5; chan++) {
                    averageRaw[chan][ppmAverageIndex % 4] = (pulsevals[chan] - 1000) / 500.f - 1;
                    rawvals[chan] = 0;
                    for (uint8_t i = 0; i < 4; i++)
                        rawvals[chan] += averageRaw[chan][i];
                    rawvals[chan] /= 4;
                }
                ppmAverageIndex++;
            }
    };
}
