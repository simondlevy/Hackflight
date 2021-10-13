/* 
   Utilitiy functions

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <math.h>
#include <stdint.h>

#ifndef M_PI
static const float M_PI = 3.141593;
#endif

namespace hf {

    static float constrainMinMax(float val, float min, float max)
    {
        return (val<min) ? min : ((val>max) ? max : val);
    }

    static float constrainAbs(float val, float max)
    {
        return constrainMinMax(val, -max, +max);
    }

    static void quat2euler(float qw, float qx, float qy, float qz,
            float & ex, float & ey, float & ez)
    {
        ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
        ey = asin(2.0f*(qx*qz-qw*qy));
        ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
    }

    static float deg2rad(float degrees)
    {
        return degrees * M_PI / 180;
    }

} // namespace hf
