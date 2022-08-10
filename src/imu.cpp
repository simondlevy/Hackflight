/*
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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "macros.h"

#include "arming.h"
#include "core_rate.h"
#include "deg2rad.h"
#include "maths.h"
#include "system.h"
#include "time.h"

// Constants for trig functions

static const float atanPolyCoef1  = 3.14551665884836e-07f;
static const float atanPolyCoef2  = 0.99997356613987f;
static const float atanPolyCoef3  = 0.14744007058297684f;
static const float atanPolyCoef4  = 0.3099814292351353f;
static const float atanPolyCoef5  = 0.05030176425872175f;
static const float atanPolyCoef6  = 0.1471039133652469f;
static const float atanPolyCoef7  = 0.6444640676891548f;

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

static void quat2euler(quaternion_t * quat, vehicle_state_t * state, rotation_t * rot)
{
    float qw = quat->w;
    float qx = quat->x;
    float qy = quat->y;
    float qz = quat->z;

    float r00 = 1 - 2 * qy*qy - 2 * qz*qz;
    float r10 = 2 * (qx*qy + qw*qz);
    float r20 = 2 * (qx*qz - qw*qy);
    float r21 = 2 * (qy*qz + qw*qx);
    float r22 = 1 - 2 * qx*qx - 2 * qy*qy;

    float psi = -atan2_approx(r10, r00); 

    // Results
    state->phi   = atan2_approx(r21, r22); 
    state->theta = (0.5f * M_PI) - acos_approx(-r20);
    state->psi   = psi + ((psi < 0) ? 2 * M_PI : 0);

    // Additional output
    rot->r20 = r20;
    rot->r21 = r21;
    rot->r22 = r22;
}

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

static void update(
        gyro_t * gyro,
        imu_fusion_t * fusionPrev,
        uint32_t time,
        quaternion_t * quat,
        rotation_t * rot)
{
    imu_fusion_t fusion;
    fusion.time = time;
    memcpy(&fusion.quat, quat, sizeof(quaternion_t));
    memcpy(&fusion.rot, rot, sizeof(rotation_t));
    memcpy(fusionPrev, &fusion, sizeof(imu_fusion_t));
    memset(&gyro->accum, 0, sizeof(imu_sensor_t));
}

static void getQuaternion(
        gyro_t * gyro,
        arming_t * arming,
        imu_fusion_t * fusionPrev,
        uint32_t time,
        quaternion_t * quat)
{
    int32_t deltaT = time - fusionPrev->time;

    axes_t gyroAvg = {};
    getAverage(&gyro->accum, CORE_PERIOD(), &gyroAvg);

    float dt = deltaT * 1e-6;

    gyro_reset_t new_gyro_reset = {};

    if (!armingIsArmed(arming)) {
        memcpy(&fusionPrev->gyroReset, &new_gyro_reset, sizeof(gyro_reset_t));
    }

    mahony(dt, &gyroAvg, &fusionPrev->quat, quat);
}

// ----------------------------------------------------------------------------

void imuAccumulateGyro(gyro_t * gyro)
{
    static float _adcf[3];

    // integrate using trapezium rule to avoid bias
    gyro->accum.values.x +=
        0.5f * (_adcf[0] + gyro->dps_filtered[0]) * CORE_PERIOD();
    gyro->accum.values.y +=
        0.5f * (_adcf[1] + gyro->dps_filtered[1]) * CORE_PERIOD();
    gyro->accum.values.z +=
        0.5f * (_adcf[2] + gyro->dps_filtered[2]) * CORE_PERIOD();

    gyro->accum.count++;

    for (int axis = 0; axis < 3; axis++) {
        _adcf[axis] = gyro->dps_filtered[axis];
    }
}

void imuGetEulerAngles(
        hackflight_t * hf,
        gyro_t * gyro,
        imu_fusion_t * fusionPrev,
        uint32_t time)
{
    quaternion_t quat = {0,0,0,0};

    getQuaternion(gyro, &hf->arming, fusionPrev, time, &quat);

    rotation_t rot = {0,0,0};

    quat2euler(&quat, &hf->vstate, &rot);

    update(gyro, fusionPrev, time, &quat, &rot);
}
