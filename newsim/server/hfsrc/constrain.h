/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

static inline float constrain_f(const float amt, const float low, const float high)
{
    return amt < low ? low : amt > high ? high : amt;
}

static inline uint16_t constrain_u16(const uint16_t amt, const uint16_t low, const uint16_t high)
{
    return amt < low ? low : amt > high ? high : amt;
}

static inline int32_t constrain_f_i32(const float amt, const int32_t low, const int32_t high)
{
    return amt < low ? low : amt > high ? high : amt;
}

static inline int32_t constrain_i32_u32(const int32_t amt, const uint32_t low, const uint32_t high)
{
    if (amt < (int32_t)low)
        return low;
    else if (amt > (int32_t)high)
        return high;
    else
        return amt;
}

