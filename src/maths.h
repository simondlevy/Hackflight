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

#include <math.h>
#include <stdint.h>

typedef struct fp_rotationMatrix_s {
    float m[3][3];              // matrix
} fp_rotationMatrix_t;

#if defined(__cplusplus)
extern "C" {
#endif

    float sin_approx(float x);
    float cos_approx(float x);
    float atan2_approx(float y, float x);
    float acos_approx(float x);

    static inline int constrain(int amt, int low, int high)
    {
        if (amt < low)
            return low;
        else if (amt > high)
            return high;
        else
            return amt;
    }

    static inline float constrainf(float amt, float low, float high)
    {
        if (amt < low)
            return low;
        else if (amt > high)
            return high;
        else
            return amt;
    }

    static void alignSensorViaRotation(float *dest)
    {
        const float x = dest[0];
        const float y = dest[1];
        const float z = dest[2];

        // 270 degrees
        dest[0] = -y;
        dest[1] = x;
        dest[2] = z;
    }

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float sq(float x)
{
    return x * x;
}

#if defined(__cplusplus)
}
#endif
