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
#include "quat2euler.h"
#include "time.h"


#if defined(__cplusplus)
extern "C" {
#endif

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float square(float x)
{
    return x * x;
}

static void getAverage(imu_sensor_t * sensor, uint32_t period, axes_t * avg)
{
    uint32_t denom = sensor->count * period;

    avg->x = denom ? sensor->values.x / denom : 0;
    avg->y = denom ? sensor->values.y / denom : 0;
    avg->z = denom ? sensor->values.z / denom : 0;
}

static void mahony(
        float dt,
        axes_t * gyro,
        quaternion_t * quat_old,
        quaternion_t * quat_new)
{
    // Convert gyro degrees to radians
    float gx = deg2rad(gyro->x);
    float gy = deg2rad(gyro->y);
    float gz = deg2rad(gyro->z);

    // Apply proportional and integral feedback, then integrate rate-of-change
    float gx1 = gx * dt / 2;
    float gy1 = gy * dt / 2;
    float gz1 = gz * dt / 2;

    // Update quaternion
    float qw =
        quat_old->w - quat_old->x * gx1 - quat_old->y * gy1 - quat_old->z * gz1;
    float qx =
        quat_old->x + quat_old->w * gx1 + quat_old->y * gz1 - quat_old->z * gy1;
    float qy =
        quat_old->y + quat_old->w * gy1 - quat_old->x * gz1 + quat_old->z * gx1;
    float qz =
        quat_old->z + quat_old->w * gz1 + quat_old->x * gy1 - quat_old->y * gx1;

    // Normalise quaternion
    float recipNorm = invSqrt(square(qw) + square(qx) + square(qy) + square(qz));
    quat_new->w = qw * recipNorm;
    quat_new->x = qx * recipNorm;
    quat_new->y = qy * recipNorm;
    quat_new->z = qz * recipNorm;
}
// =============================================================================



int32_t imuGetGyroSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles)
{
    int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyroSyncTime()) % desiredPeriodCycles;

    if (gyroSkew > (desiredPeriodCycles / 2)) {
        gyroSkew -= desiredPeriodCycles;
    }

    return gyroSkew;
}

static void getQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
{
    int32_t deltaT = time - hf->imuFusionPrev.time;

    axes_t gyroAvg = {0,0,0};
    getAverage(&hf->gyro.accum, CORE_PERIOD(), &gyroAvg);

    float dt = deltaT * 1e-6;

    imu_fusion_t * fusionPrev = &hf->imuFusionPrev;

    gyro_reset_t new_gyro_reset = {0};

    if (!armingIsArmed(&hf->arming)) {
        memcpy(&fusionPrev->gyroReset, &new_gyro_reset, sizeof(gyro_reset_t));
    }

    mahony(dt, &gyroAvg, &fusionPrev->quat, quat);
}

void updateFusion(hackflight_t * hf, uint32_t time, quaternion_t * quat, rotation_t * rot)
{
    imu_fusion_t fusion;
    fusion.time = time;
    memcpy(&fusion.quat, quat, sizeof(quaternion_t));
    memcpy(&fusion.rot, rot, sizeof(rotation_t));
    memcpy(&hf->imuFusionPrev, &fusion, sizeof(imu_fusion_t));
    memset(&hf->gyro.accum, 0, sizeof(imu_sensor_t));
}

void imuGetEulerAngles(hackflight_t * hf, uint32_t time)
{
    quaternion_t quat = {0,0,0,0};

    getQuaternion(hf, time, &quat);

    rotation_t rot = {0,0,0};

    quat2euler(&quat, &hf->vstate, &rot);

    updateFusion(hf, time, &quat, &rot);
}

#if defined(__cplusplus)
}
#endif


