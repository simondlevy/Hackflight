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
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "time.h"

#if defined(__cplusplus)
extern "C" {
#endif

void imuAccumulateGyro(gyro_t * gyro)
{
    static axes_t _adcf;

    // integrate using trapezium rule to avoid bias
    gyro->accum.values.x +=
        0.5f * (_adcf.x + gyro->x.dpsFiltered) * CORE_PERIOD();
    gyro->accum.values.y +=
        0.5f * (_adcf.y + gyro->y.dpsFiltered) * CORE_PERIOD();
    gyro->accum.values.z +=
        0.5f * (_adcf.z + gyro->z.dpsFiltered) * CORE_PERIOD();

    gyro->accum.count++;

    _adcf.x = gyro->x.dpsFiltered;
    _adcf.y = gyro->y.dpsFiltered;
    _adcf.z = gyro->z.dpsFiltered;
}

int32_t imuGetGyroSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles)
{
    int32_t gyroSkew =
        cmpTimeCycles(nextTargetCycles, gyroSyncTime()) % desiredPeriodCycles;

    if (gyroSkew > (desiredPeriodCycles / 2)) {
        gyroSkew -= desiredPeriodCycles;
    }

    return gyroSkew;
}

#if defined(__cplusplus)
}
#endif


