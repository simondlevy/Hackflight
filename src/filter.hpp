/* 
   filter.hpp: Filtering functions

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

#include "debug.hpp"

namespace hf {
 
class Filter {

    public:
        
        static float deadband(float value, float deadband);
        static float complementary(float a, float b, float c);
        static float constrainMinMax(float val, float min, float max);
        static float constrainAbs(float val, float max);
};

/********************************************* CPP ********************************************************/

float Filter::deadband(float value, float deadband)
{
    if (fabs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

float Filter::complementary(float a, float b, float c)
{
    return a * c + b * (1 - c);
}

float Filter::constrainMinMax(float val, float min, float max)
{
    return (val<min) ? min : ((val>max) ? max : val);
}

float Filter::constrainAbs(float val, float max)
{
    return constrainMinMax(val, -max, +max);
}

} // namespace hf
