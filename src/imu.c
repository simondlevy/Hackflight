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

#include "accel.h"
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

// Other constants

// 500ms - Time to wait for attitude to converge at high gain
static const uint32_t  ATTITUDE_RESET_ACTIVE_TIME = 500000;  

// dcmKpGain value to use during attitude reset
static const float ATTITUDE_RESET_KP_GAIN = 2;       

// 15 deg/sec - gyro limit for quiet period
static const float ATTITUDE_RESET_GYRO_LIMIT  = 15;      

// 250ms - gyro quiet period after disarm before attitude reset
static const uint32_t ATTITUDE_RESET_QUIET_TIME  = 250000;  

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

static bool bigGyro(float val)
{
    return fabsf(val) > ATTITUDE_RESET_GYRO_LIMIT;
}

// DCM filter proportional gain ( x 10000)
static const float DCM_KP = 0.25; 

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
        axes_t * accel,
        bool useAcc,
        float dcmKpGain,
        rotation_t * rot,
        quaternion_t * quat_old,
        quaternion_t * quat_new)
{
    // Convert gyro degrees to radians
    float gx = deg2rad(gyro->x);
    float gy = deg2rad(gyro->y);
    float gz = deg2rad(gyro->z);

    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;

    // Use measured acceleration vector
    float recipAccNorm = square(ax) + square(ay) + square(az);

    bool goodAcc = useAcc && recipAccNorm > 0.01;

    float recipAccNormInv = goodAcc ? invSqrt(recipAccNorm) : 0;

    // Normalise accelerometer measurement
    float ax1 = goodAcc ? ax * recipAccNormInv : ax;
    float ay1 = goodAcc ? ay * recipAccNormInv : ay;
    float az1 = goodAcc ? az * recipAccNormInv : az;

    // Raw heading error is sum of cross product between estimated direction
    // and measured direction of gravity
    float ex = goodAcc ? ay1 * rot->r22 - az1 * rot->r21 : 0;
    float ey = goodAcc ? az1 * rot->r20 - ax1 * rot->r22 : 0;
    float ez = goodAcc ? ax1 * rot->r21 - ay1 * rot->r20 : 0;

    // Apply proportional and integral feedback, then integrate rate-of-change
    float gx1 = (gx + dcmKpGain * ex) * dt / 2;
    float gy1 = (gy + dcmKpGain * ey) * dt / 2;
    float gz1 = (gz + dcmKpGain * ez) * dt / 2;

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

static bool isAccelHealthy(axes_t *accAvg)
{
    float accMagnitudeSq =
        square(accAvg->x) + square(accAvg->y) + square(accAvg->z) * square(1.0f/ACCEL_1G);

    // Accept accel readings only in range 0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}

static bool checkReset(
        uint32_t currentTimeUs,
        bool useAcc,
        axes_t *gyroAvg,
        bool armed,
        gyro_reset_t * status_old,
        gyro_reset_t * status_new)
{
    // Just disarmed; start the gyro quiet period
    uint32_t quietPeriodEnd = armed ?
        currentTimeUs + ATTITUDE_RESET_QUIET_TIME :
        status_old->quietPeriodEnd; 

    uint32_t resetTimeEnd = armed ? 0 : status_old->resetTimeEnd;

    bool resetCompleted = armed ? false : status_old->resetCompleted;

    bool tmp = (resetTimeEnd > 0 || quietPeriodEnd > 0 || resetCompleted) &&
        (bigGyro(gyroAvg->x) || bigGyro(gyroAvg->y) || bigGyro(gyroAvg->z) || !useAcc);

    uint32_t quietPeriodEnd1 = tmp ? 
        currentTimeUs + ATTITUDE_RESET_QUIET_TIME :
        quietPeriodEnd;

    uint32_t resetTimeEnd1 = tmp ? 0 : resetTimeEnd;

    bool tmp1 = resetTimeEnd1 > 0;
    bool tmp2 = currentTimeUs >= resetTimeEnd1;
    bool tmp3 = quietPeriodEnd1 > 0 && currentTimeUs >= quietPeriodEnd1;
    bool tmp4 = tmp1 && tmp2;

    status_new->resetCompleted = tmp4 ? true : resetCompleted;

    status_new->quietPeriodEnd = tmp4 || tmp3 ? 0 : quietPeriodEnd1;

    status_new->resetTimeEnd = tmp1 ?
        (tmp2 ? 0 : resetTimeEnd) :
        tmp3 ?
        currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME :
        resetTimeEnd1;

    return tmp1 && !tmp2;
}

// =============================================================================

void imuAccelTask(void * hackflight, uint32_t time)
{
    (void)time;

    hackflight_t * hf = (hackflight_t *)hackflight;

    accelUpdate(hf->imuAlignFun, &hf->accelAccum);
}

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


static void getQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
{
    int32_t deltaT = time - hf->imuFusionPrev.time;

    axes_t gyroAvg = {0,0,0};
    getAverage(&hf->gyro.accum, CORE_PERIOD(), &gyroAvg);

    axes_t accelAvg = {0,0,0};
    getAverage(&hf->accelAccum, 1, &accelAvg);

    bool useAcc = isAccelHealthy(&accelAvg);

    float dt = deltaT * 1e-6;

    imu_fusion_t * fusionPrev = &hf->imuFusionPrev;

    gyro_reset_t new_gyro_reset = {0};

    bool resetActive1 = checkReset(
            time,
            useAcc,
            &gyroAvg,
            armingIsArmed(&hf->arming),
            &fusionPrev->gyroReset,
            &new_gyro_reset);

    bool resetActive = armingIsArmed(&hf->arming) ?  false : resetActive1;

    if (!armingIsArmed(&hf->arming)) {
        memcpy(&fusionPrev->gyroReset, &new_gyro_reset, sizeof(gyro_reset_t));
    }

    // Scale the kP to generally converge faster when disarmed.
    float kpgain = resetActive ?
        ATTITUDE_RESET_KP_GAIN :
        DCM_KP * (!armingIsArmed(&hf->arming) ? 10 : 1);

    mahony(dt, &gyroAvg, &accelAvg, useAcc, kpgain,
            &fusionPrev->rot, &fusionPrev->quat, quat);
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


