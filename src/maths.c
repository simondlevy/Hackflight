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

#include <stdint.h>
#include <math.h>

#include "maths.h"

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06

static const float sinPolyCoef3 = -1.666568107e-1f;
static const float sinPolyCoef5 =  8.312366210e-3f;
static const float sinPolyCoef7 = -1.849218155e-4f;
static const float sinPolyCoef9 =  0;

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;  // Stop here on error input (5 * 360 Deg)
    while (x >  M_PI) x -= (2.0f * M_PI);      // always wrap input angle to -PI..PI
    while (x < -M_PI) x += (2.0f * M_PI);
    if (x >  (0.5f * M_PI)) x =  (0.5f * M_PI) - (x - (0.5f * M_PI)); // just pick -90..+90 deg
    else if (x < -(0.5f * M_PI)) x = -(0.5f * M_PI) - ((0.5f * M_PI) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 *
                (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PI));
}
