/*
   stabilize_multiwii.hpp : Class declaration for old-school Multiwii PID-based stablization

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

        public:

            void init(class RC * _rc, class IMU * _imu);

            void update(void);

            void resetIntegral(void);

            int16_t axisPID[3];

        private:

            class RC  * rc;
            class IMU * imu;

            uint8_t rate_p[3];
            uint8_t rate_i[3];
            uint8_t rate_d[3];

            int16_t lastGyroError[3];
            int32_t delta1[3]; 
            int32_t delta2[3];
            int32_t errorGyroI[3];
            int32_t errorAngleI[2];
    }; 

#ifdef __arm__
} // extern "C"
#endif
