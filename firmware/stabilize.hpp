/*
   stabilize.hpp : Class declaration for PID-based stablization

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

#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

#ifdef __arm__
extern "C" {
#endif

    class Stabilize {

        protected:

            class RC  * rc;
            class IMU * imu;

        public:

            int16_t axisPID[3];

            void init(class RC * _rc, class IMU * _imu)  { this->rc = _rc; this->imu = _imu; }

#ifndef __arm__
            
            virtual void update(void) = 0;

            virtual void resetIntegral(void) = 0;
#endif
    }; 

#ifdef __arm__
} // extern "C"
#endif
