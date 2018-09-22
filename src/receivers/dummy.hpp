/*
   dummy.hpp : "Dummy" receiver subclass for prototyping

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

namespace hf {

    class Dummy_Receiver : public Receiver {

        protected:

            void begin(void)
            {
            }

            bool gotNewFrame(void)
            {
                return false;
            }

            void readRawvals(void)
            {
            }

            bool lostSignal(void)
            {
                return false;
            }

        public:

            Dummy_Receiver(const uint8_t channelMap[6]) :  Receiver(channelMap) 
            { 
            }

    }; // class Dummy_Receiver

} // namespace hf
