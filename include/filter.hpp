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

namespace hf {
 
class Filter {

    public:
        
        static int32_t deadband(int32_t value, int32_t deadband);
        static float   complementary(float a, float b, float c);
        static float   max(float a, float b);
        static int32_t constrainMinMax(int32_t val, int32_t min, int32_t max);
        static int32_t constrainAbs(int32_t val, int32_t max);
};

/********************************************* CPP ********************************************************/

int32_t Filter::deadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
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

float Filter::max(float a, float b)
{
    return a > b ? a : b;
}

int32_t Filter::constrainMinMax(int32_t val, int32_t min, int32_t max)
{
    return (val<min) ? min : ((val>max) ? max : val);
}

int32_t Filter::constrainAbs(int32_t val, int32_t max)
{
    return constrainMinMax(val, -max, +max);
}

} // namespace hf
