/*
   mock.hpp : "Mock" receiver subclass for prototyping

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

static constexpr uint8_t DEFAULT_MAP[6] = {0,1,2,3,4,5};

namespace hf {

    class MockReceiver : public Receiver {

        protected:

            void begin(void)
            {
            }

            virtual bool gotNewFrame(void) override
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

            MockReceiver(void) 
                : Receiver(DEFAULT_MAP)
            { 
            }

    }; // class MockReceiver

} // namespace hf
