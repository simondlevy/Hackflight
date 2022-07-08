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
#include "quat2euler.h"
#include "time.h"

// deg/sec - gyro limit for quiet period
static const float ATTITUDE_RESET_GYRO_LIMIT = 15;

//  Time (usec) to wait for attitude to converge at high gain
static const float ATTITUDE_RESET_ACTIVE_TIME = 500000;  

// gyro quiet period after disarm before attitude reset
static const uint32_t ATTITUDE_RESET_QUIET_TIME = 250000;   

static const float DCM_KP = 0.25;
static const float DCM_KI = 0.03;

// dcmKpGain value to use during attitude reset
static const float ATTITUDE_RESET_KP_GAIN = 25;

static const float SPIN_RATE_LIMIT = 20; // deg/s

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float square(float x)
{
    return x * x;
}

static void getAverage(imuSensor_t * sensor, uint32_t period, axes_t * avg)
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
        quaternion_t * quat_old,
        rotation_t * rot,
        float dcmKpGain,
        quaternion_t * quat_new)
{
    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;

    // Integral error terms scaled by Ki
    static float integralFBx,  integralFBy, integralFBz;    

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;

    // Use measured acceleration vector
    float recipAccNorm = square(ax) + square(ay) + square(az);
    if (recipAccNorm > 0.01) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and
        // measured direction of gravity
        ex += (ay * rot->r22 - az * rot->r21);
        ey += (az * rot->r20 - ax * rot->r22);
        ez += (ax * rot->r21 - ay * rot->r20);
    }

    // Convert gyro degrees to radians
    float gx = deg2rad(gyro->x);
    float gy = deg2rad(gyro->y);
    float gz = deg2rad(gyro->z);

    // Compute and apply integral feedback if enabled
    if (DCM_KI > 0) {

        // Calculate general spin rate (rad/s)
        float spin_rate = sqrtf(square(gx) + square(gy) + square(gz));

        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < deg2rad(SPIN_RATE_LIMIT)) {
            integralFBx += DCM_KI * ex * dt;    // integral error scaled by Ki
            integralFBy += DCM_KI * ey * dt;
            integralFBz += DCM_KI * ez * dt;
        }
    } else {
        integralFBx = 0;    // prevent integral windup
        integralFBy = 0;
        integralFBz = 0;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= dt / 2;
    gy *= dt / 2;
    gz *= dt / 2;

    // Update quaternion
    float qw =
        quat_old->w - quat_old->x * gx - quat_old->y * gy - quat_old->z * gz;
    float qx =
        quat_old->x + quat_old->w * gx + quat_old->y * gz - quat_old->z * gy;
    float qy =
        quat_old->y + quat_old->w * gy - quat_old->x * gz + quat_old->z * gx;
    float qz =
        quat_old->z + quat_old->w * gz + quat_old->x * gy - quat_old->y * gx;

    // Normalise quaternion
    float recipNorm = invSqrt(square(qw) + square(qx) + square(qy) + square(qz));
    quat_new->w = qw * recipNorm;
    quat_new->x = qx * recipNorm;
    quat_new->y = qy * recipNorm;
    quat_new->z = qz * recipNorm;
}

// =============================================================================

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

// Calculate the dcmKpGain to use. When armed, the gain is
// imuRuntimeConfig.dcm_kp * 1.0 scaling.  When disarmed after initial boot,
// the scaling is set to 10.0 for the first 20 seconds to speed up initial
// convergence.  After disarming we want to quickly reestablish convergence to
// deal with the attitude estimation being incorrect due to a crash.  - wait
// for a 250ms period of low gyro activity to ensure the craft is not moving -
// use a large dcmKpGain value for 500ms to allow the attitude estimate to
// quickly converge - reset the gain back to the standard setting
static float calculateKpGain(uint32_t time, axes_t * gyroAvg, bool armState)
{
    static bool lastArmState;
    static uint32_t gyroQuietPeriodTimeEnd;
    static uint32_t attitudeResetTimeEnd;
    static bool attitudeResetCompleted;

    float ret = 0;

    bool attitudeResetActive = false;

    if (!armState) {
        if (lastArmState) {   // Just disarmed; start the gyro quiet period
            gyroQuietPeriodTimeEnd = time + ATTITUDE_RESET_QUIET_TIME;
            attitudeResetTimeEnd = 0;
            attitudeResetCompleted = false;
        }

        // If gyro activity exceeds the threshold then restart the quiet
        // period.  Also, if the attitude reset has been complete and there is
        // subsequent gyro activity then start the reset cycle again. This
        // addresses the case where the pilot rights the craft after a crash.
        if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) ||
                attitudeResetCompleted) {
            if ((fabsf(gyroAvg->x) > ATTITUDE_RESET_GYRO_LIMIT)
                    || (fabsf(gyroAvg->y) > ATTITUDE_RESET_GYRO_LIMIT)
                    || (fabsf(gyroAvg->z) > ATTITUDE_RESET_GYRO_LIMIT)) {

                gyroQuietPeriodTimeEnd = time + ATTITUDE_RESET_QUIET_TIME;
                attitudeResetTimeEnd = 0;
            }
        }

        // Resetting the attitude estimation
        if (attitudeResetTimeEnd > 0) {        
            if (time >= attitudeResetTimeEnd) {
                gyroQuietPeriodTimeEnd = 0;
                attitudeResetTimeEnd = 0;
                attitudeResetCompleted = true;
            } else {
                attitudeResetActive = true;
            }
        } else if ((gyroQuietPeriodTimeEnd > 0) &&
                (time >= gyroQuietPeriodTimeEnd)) {
            // Start the high gain period to bring the estimation into convergence
            attitudeResetTimeEnd = time + ATTITUDE_RESET_ACTIVE_TIME;
            gyroQuietPeriodTimeEnd = 0;
        }
    }
    lastArmState = armState;

    if (attitudeResetActive) {
        ret = ATTITUDE_RESET_KP_GAIN;
    } else {
        ret = DCM_KP;
        if (!armState) {
            ret *= 10; // Scale the kP to converge faster when disarmed
        }
    }

    return ret;
}


void imuGetEulerAngles(hackflight_t * hf, uint32_t time, axes_t * angles)
{
    int32_t deltaT = time - hf->imuFusionPrev.time;

    axes_t gyroAvg = {0,0,0};
    getAverage(&hf->gyro.accum, CORE_PERIOD(), &gyroAvg);

    float dt = deltaT * 1e-6;

    imuFusion_t * fusionPrev = &hf->imuFusionPrev;

    gyroReset_t new_gyro_reset = {0,0,false};

    if (!armingIsArmed(&hf->arming)) {
        memcpy(&fusionPrev->gyroReset, &new_gyro_reset, sizeof(gyroReset_t));
    }

    static rotation_t rot;

    axes_t accelRaw = hf->accel.raw;

    

    quaternion_t quat = {0,0,0,0};
    mahony(
            dt,
            &gyroAvg,
            &accelRaw,
            &fusionPrev->quat,
            &rot, 
            calculateKpGain(time, &gyroAvg, hf->arming.is_armed),        
            &quat);

    quat2euler(&quat, angles, &rot);

    imuFusion_t fusion;
    fusion.time = time;
    memcpy(&fusion.quat, &quat, sizeof(quaternion_t));
    memcpy(&fusion.rot, &rot, sizeof(rotation_t));
    memcpy(&hf->imuFusionPrev, &fusion, sizeof(imuFusion_t));
    memset(&hf->gyro.accum, 0, sizeof(imuSensor_t));
}
