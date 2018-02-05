/*
   board.hpp : class header for board-specific routines

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

#include <stdarg.h>

#include "datatypes.hpp"

namespace hf {

    class Board {

        public:

            //------------------------------------ Core functionality ----------------------------------------------------
            virtual void     init(void) = 0;
            virtual void     getState(vehicle_state_t & state) = 0;
            virtual uint32_t getMicroseconds() = 0;
            virtual void     writeMotor(uint8_t index, float value) = 0;

            //----------------------------------------- Safety ----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }

            //---------------------------------- Serial communications  -------------------------------------------------
            virtual void doSerialComms(vehicle_state_t * state, class Receiver * receiver, class Mixer * mixer)  
            { (void)state; (void)receiver; (void)mixer; }

            //--------------------------------------- Debugging ---------------------------------------------------------
            static void  outbuf(char * buf);

    }; // class Board

} // namespace
