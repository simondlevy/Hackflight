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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "arming.h"
#include "core_rate.h"
#include "datatypes.h"
#include "deg2rad.h"
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "time.h"

// Constants for trig functions

static const float atanPolyCoef1  = 3.14551665884836e-07f;
static const float atanPolyCoef2  = 0.99997356613987f;
static const float atanPolyCoef3  = 0.14744007058297684f;
static const float atanPolyCoef4  = 0.3099814292351353f;
static const float atanPolyCoef5  = 0.05030176425872175f;
static const float atanPolyCoef6  = 0.1471039133652469f;
static const float atanPolyCoef7  = 0.6444640676891548f;

#if defined(__cplusplus)
extern "C" {
#endif

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
static float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) *
        (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PI - result;
    else
        return result;
}

static float atan2_approx(float y, float x)
{
    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = absX > absY ? absX : absY;
    if (res) res = (absX < absY ? absX : absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) *
                res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 *
                    res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PI / 2.0f) - res;
    if (x < 0) res = M_PI - res;
    if (y < 0) res = -res;
    return res;
}

// =============================================================================

void imuAccumulateGyro(gyro_t * gyro)
{
    static float _adcf[3];

    // integrate using trapezium rule to avoid bias
    gyro->accum.values.x += 0.5f * (_adcf[0] + gyro->dps_filtered[0]) * CORE_PERIOD();
    gyro->accum.values.y += 0.5f * (_adcf[1] + gyro->dps_filtered[1]) * CORE_PERIOD();
    gyro->accum.values.z += 0.5f * (_adcf[2] + gyro->dps_filtered[2]) * CORE_PERIOD();

    gyro->accum.count++;

    for (int axis = 0; axis < 3; axis++) {
        _adcf[axis] = gyro->dps_filtered[axis];
    }
}

int32_t imuGetGyroSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles)
{
    int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyroSyncTime()) % desiredPeriodCycles;

    if (gyroSkew > (desiredPeriodCycles / 2)) {
        gyroSkew -= desiredPeriodCycles;
    }

    return gyroSkew;
}

#if defined(__cplusplus)
}
#endif


