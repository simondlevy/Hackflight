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
#include "datatypes.h"
#include "deg2rad.h"
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "quat2euler.h"
#include "time.h"

// =============================================================================

void imuAccelTask(void * hackflight, uint32_t time)
{
    (void)time;

    hackflight_t * hf = (hackflight_t *)hackflight;

    accelUpdate(&hf->accelAccum);
}

void imuAccumulateGyro(hackflight_t * hf, float * adcf)
{
    static float _adcf[3];

    // integrate using trapezium rule to avoid bias
    hf->gyroAccum.values.x += 0.5f * (_adcf[0] + adcf[0]) * GYRO_PERIOD();
    hf->gyroAccum.values.y += 0.5f * (_adcf[1] + adcf[1]) * GYRO_PERIOD();
    hf->gyroAccum.values.z += 0.5f * (_adcf[2] + adcf[2]) * GYRO_PERIOD();

    hf->gyroAccum.count++;

    for (int axis = 0; axis < 3; axis++) {
        _adcf[axis] = adcf[axis];
    }
}

void imuGetEulerAngles(hackflight_t * hf, uint32_t time)
{
    quaternion_t quat = {0};

    imuGetQuaternion(hf, time, &quat);

    rotation_t rot = {0};

    quat2euler(&quat, &hf->vstate, &rot);

    imuUpdateFusion(hf, time, &quat, &rot);
}

void imuUpdateFusion(hackflight_t * hf, uint32_t time, quaternion_t * quat, rotation_t * rot)
{
    imu_fusion_t fusion;
    fusion.time = time;
    memcpy(&fusion.quat, quat, sizeof(quaternion_t));
    memcpy(&fusion.rot, rot, sizeof(rotation_t));
    memcpy(&hf->imuFusionPrev, &fusion, sizeof(imu_fusion_t));
    memset(&hf->gyroAccum, 0, sizeof(imu_sensor_t));
}
