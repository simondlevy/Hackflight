/*
   IMU class

   Copyright (c) 2020 Simon D. Levy

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

namespace hf {

    class IMU {

        // NB: quaternion, gyrometer, accelerometer should return values as follows:
        //
        // QW: rightside-up +1, upside-down 0
        // QX: roll right +,    left -
        // QY: pitch forward -, back +
        // QZ: yaw left -,      right +
        //
        // GX: roll right +,    left -
        // GY: pitch forward -, back +
        // GZ: yaw right +,     left -
        //
        // AX: pitch forward +, back -
        // AY: roll right +,    left -
        // AZ: rightside-up +,  upside-down -

        friend class Hackflight;
        friend class Quaternion;
        friend class Gyrometer;

        protected:

            // Core functionality
            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) = 0;
            virtual bool getGyrometer(float & gx, float & gy, float & gz) = 0;

            // Adjustment for non-standard mounting
            virtual void adjustEulerAngles(float & x, float & y, float & z) { (void)x; (void)y; (void)z;  }
            virtual void adjustGyrometer(float & x, float & y, float & z) { (void)x; (void)y; (void)z;  }

            // Required by some IMUs
            virtual void begin(void) { }

            // Support for additional surface-mount sensors
            virtual bool  getAccelerometer(float & ax, float & ay, float & az) { (void)ax; (void)ay; (void)az; return false; }
            virtual bool  getMagnetometer(float & mx, float & my, float & mz) { (void)mx; (void)my; (void)mz; return false; }
            virtual bool  getBarometer(float & pressure) { (void)pressure;  return false; }

    }; // class IMU

} // namespace hf
