/*
   stabilize_luxfloat.cpp : LuxFloat PID-based stability class implementation

   Adapted from 

     https://github.com/multiwii/baseflight/blob/master/src/mw.c

     https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid_luxfloat.c

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

#include "hackflight.hpp"
#include "pidvals.hpp"

void StabilizeLuxFloat::init(class RC * _rc, class IMU * _imu)
{
    Stabilize::init(_rc, _imu);

}

void StabilizeLuxFloat::update(void)
{
}

void StabilizeLuxFloat::resetIntegral(void)
{
}

int32_t StabilizeLuxFloat::getRcStickDeflection(int16_t * rcData, int32_t axis, uint16_t midrc) 
{
    return min(abs(rcData[axis] - midrc), 500);
}


#ifdef __arm__
} // extern "C"
#endif
