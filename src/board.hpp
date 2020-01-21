/*
   Class header for board-specific routines

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
        friend class Debugger;
        friend class Mixer;
        friend class TimerTask;
        friend class SerialTask;

        protected:

            // NB: quaternion, gyrometer, accelerometer should return values as follows:
            //
            // QW: rightside-up +1, upside-down 0
            // QX: roll right +,    left -
            // QY: pitch forward -, back +
            // QZ: yaw left -,      right +
            //
            // GX: roll right +,    left -
            // GY: pitch forward -, back +
            // GZ: yaw left -,      right +
            //
            // AX: pitch forward +, back -
            // AY: roll right +,    left -
            // AZ: rightside-up +,  upside-down -

            //------------------------------------ Core functionality ----------------------------------------------------
            virtual bool  getQuaternion(float & qw, float & qx, float & qy, float & qz) = 0;
            virtual bool  getGyrometer(float & gx, float & gy, float & gz) = 0;
            virtual float getTime(void) = 0;

            //------------------------- Support for additional surface-mount sensors -------------------------------------
            virtual bool  getAccelerometer(float & ax, float & ay, float & az) { (void)ax; (void)ay; (void)az; return false; }
            virtual bool  getMagnetometer(float & mx, float & my, float & mz) { (void)mx; (void)my; (void)mz; return false; }
            virtual bool  getBarometer(float & pressure) { (void)pressure;  return false; }

            //------------------------------- Serial communications via MSP ----------------------------------------------
            virtual uint8_t serialAvailableBytes(void) { return 0; }
            virtual uint8_t serialReadByte(void)  { return 1; }
            virtual void    serialWriteByte(uint8_t c) { (void)c; }

            // --------------------------- Adjust IMU readings based on IMU mounting ------------------------------------
            virtual void adjustGyrometer(float & gx, float & gy, float & gz) { (void)gx; (void)gy; (void)gz; }
            virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) { (void)qw; (void)qx; (void)qy; (void)qz; }
            virtual void adjustRollAndPitch(float & roll, float & pitch) { (void)roll; (void)pitch; }

            //------------------------------- Reboot for non-Arduino boards ---------------------------------------------
            virtual void reboot(void) { }

            //----------------------------------------- Safety -----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }
            virtual void flashLed(bool shouldflash) { (void)shouldflash; }

            //--------------------------------------- Debugging ----------------------------------------------------------
            static  void outbuf(char * buf);

    }; // class Board

} // namespace
