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

        friend class Hackflight;
        friend class Quaternion;
        friend class Gyrometer;

        protected:

            // Core functionality
            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) = 0;
            virtual bool getGyrometer(float & gx, float & gy, float & gz) = 0;

            // Required by some IMUs
            virtual void begin(void) { }

    }; // class IMU

} // namespace hf
