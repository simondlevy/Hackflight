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
            virtual bool     getEulerAngles(float eulerAngles[3]) = 0;
            virtual bool     getGyroRates(float gyroRates[3]) = 0;
            virtual uint32_t getMicroseconds() = 0;
            virtual void     writeMotor(uint8_t index, float value) = 0;

            //----------------------------------- Additional PID controllers --------------------------------------------
            virtual bool     getAccelerometer(float accelGs[3]) { (void)accelGs; return false; }
            virtual bool     getBarometer(float & pressure) { (void)pressure; return false; }

            //----------------------------------------- Safety ----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }

            //---------------------------------- Serial communications  -------------------------------------------------
            virtual void doSerialComms(float eulerAngles[3], bool armed, class Receiver * receiver, class Mixer * mixer)  
            { (void)eulerAngles; (void)armed; (void)receiver; (void)mixer; }

            //--------------------------------------- Debugging ---------------------------------------------------------
            static void  outbuf(char * buf);

    }; // class Board

} // namespace
