/*
   position.cpp : Position class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mixer.c

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "mw.hpp"

class Position {

    private:

        Baro * baro;
        IMU  * imu;

        bool     velocityControl;

        int32_t  sonarAlt;
        uint32_t previousTime;
        float    accZ_old;
        float    accelVel;
        int32_t  FusedBarosonarAlt;
        int32_t  lastFusedBarosonarAlt;
        int32_t  baroAlt;
        int32_t  baroAltBaseline;
        float    accelAlt;
        bool     wasArmed;
        int32_t  baroAlt_offset;
        float    sonarTransition;

    public:

        void init(Baro * baro, IMU * imu, bool velocityControl=false);

        int32_t getAltitude(bool armed, uint32_t currentTime);
};


#ifdef __arm__
} // extern "C"
#endif
