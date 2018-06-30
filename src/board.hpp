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

namespace hf {

    class Board {

        friend class Hackflight;
        friend class Debug;
        friend class Mixer;

        protected:

            //------------------------------------ Core functionality ----------------------------------------------------
            virtual bool getQuaternion(float quat[4]) = 0;
            virtual bool getGyrometer(float gyroRates[3]) = 0;
            virtual void writeMotor(uint8_t index, float value) = 0;

            //------------------------ Support for additional PID controllers --------------------------------------------
            virtual bool getAccelerometer(float accelGs[3]) { (void)accelGs; return false; }
            virtual bool getBarometer(float & pressure) { (void)pressure; return false; }

            //------------------------------- Serial communications via MSP ----------------------------------------------
            virtual uint8_t serialAvailableBytes(void) { return 0; }
            virtual uint8_t serialReadByte(void)  { return 0; }
            virtual void    serialWriteByte(uint8_t c) { (void)c; }

            //----------------------------------------- Safety -----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }

            //--------------------------------------- Debugging ----------------------------------------------------------
            static void  outbuf(char * buf);

    }; // class Board

} // namespace
