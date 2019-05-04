/*
   board.hpp : class header for board-specific routines

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

#include <stdarg.h>
#include <stdint.h>

namespace hf {

    class Board {

        friend class Hackflight;
        friend class Gyrometer;
        friend class Quaternion;
        friend class Accelerometer;
        friend class Barometer;
        friend class Debug;
        friend class Mixer;

        protected:

            // NB: gyrometer, accelerometer should return values as follows,
            // based on the Ladybug Flight Controller:
            //
            // AX: pitch forward +, back -
            // AY: roll right +,    left -
            // AZ: rightside-up +,  upside-down -
            //
            // GX: roll right +,    left -
            // GY: pitch forward -, back +
            // GZ: yaw left -,      right +

            //------------------------------------ Core functionality ----------------------------------------------------
            virtual bool  getQuaternion(float quat[4]) = 0;
            virtual bool  getGyrometer(float gyroRates[3]) = 0;
            virtual void  writeMotor(uint8_t index, float value) = 0;
            virtual float getTime(void) = 0;

            //------------------------- Support for additional surface-mount sensors -------------------------------------
            virtual bool  getAccelerometer(float accelGs[3]) { (void)accelGs;  return false; }
            virtual bool  getMagnetometer(float uTs[3]) { (void)uTs;  return false; }
            virtual bool  getBarometer(float & pressure) { (void)pressure;  return false; }

            //------------------------------- Serial communications via MSP ----------------------------------------------
            virtual uint8_t serialAvailableBytes(void) { return 0; }
            virtual uint8_t serialReadByte(void)  { return 1; }
            virtual void    serialWriteByte(uint8_t c) { (void)c; }

            //------------------------------- Reboot for non-Arduino boards ---------------------------------------------
            virtual void reboot(void) { }

            //----------------------------------------- Safety -----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }
            virtual void flashLed(bool shouldflash) { (void)shouldflash; }
            virtual bool isBatteryLow(void) { return false; }

            //--------------------------------------- Debugging ----------------------------------------------------------
            virtual void getRawImu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz) 
                {(void)ax; (void)ay; (void)az; (void)gx; (void)gy; (void)gz; }
            static  void outbuf(char * buf);

    }; // class Board

} // namespace
