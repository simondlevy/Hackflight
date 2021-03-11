/*
   Abstract class for Open-Loop Controllers 

   Copyright (c) 2021 Simon D. Levy

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

#include <stdint.h>
#include <math.h>

#include "demands.hpp"

namespace hf {

    class OpenLoopController {

        friend class Hackflight;
        friend class SerialTask;
        friend class PidTask;

        protected: 

            virtual void getDemands(demands_t & demands) = 0;

            virtual void begin(void) 
            { 
            }

            virtual float getRawval(uint8_t chan)
            {
                return 0;
            }

            virtual bool lostSignal(void) 
            { 
                return false; 
            }

            virtual bool ready(void)
            {
                return true;
            }

            virtual bool inactive(void) 
            {
                return false;
            }

            virtual bool inArmedState(void)
            {
                return true;
            }

            virtual uint8_t getModeIndex(void)
            {
                return 0;
            }

    }; // class OpenLoopController

} // namespace
