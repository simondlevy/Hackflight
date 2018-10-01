/*
   sbus.hpp : Futaba SBUS receiver support for Arduino flight controllers

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
#include <SBUSRX.h>

static SBUSRX rx;


namespace hf {

    class SBUS_Receiver : public Receiver {

        private:

            // These values must persist between calls to readRawvals()
            float channels[16];

            const uint16_t MAX_FAILSAFE = 10;

            uint16_t failsafeCount;

        protected:

            bool gotNewFrame(void)
            {
                uint8_t failsafe = 0;
                uint16_t lostFrames = 0;

                if (rx.getChannelValuesNormalized(channels, &failsafe, &lostFrames)) {

                    // accumulate consecutive failsafe hits
                    if (failsafe) {
                        failsafeCount++;
                    }
                    else { // reset count
                        failsafeCount = 0;
                    }

                    return true;
                }

                return false;
            }

            void readRawvals(void)
            {
                memset(rawvals, 0, CHANNELS*sizeof(float));
                memcpy(rawvals, channels, CHANNELS*sizeof(float));
            }

            bool lostSignal(void)
            {
                return failsafeCount > MAX_FAILSAFE;
            }

        public:

            SBUS_Receiver(const uint8_t channelMap[6]) :  Receiver(channelMap) 
            { 
                failsafeCount = 0;
            }

    }; // class SBUS_Receiver

} // namespace
