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

#include <stdbool.h>
#include <stdint.h>

#include "datatypes.h"
#include "time.h"

typedef struct {

    uint32_t quietPeriodEnd;
    uint32_t resetTimeEnd;
    bool resetCompleted;

} gyro_reset_t;

typedef struct {

    uint32_t time;
    quaternion_t quat;
    rotation_t rot;
    gyro_reset_t gyroReset;

} imu_fusion_t;

#if defined(__cplusplus)
extern "C" {
#endif

    void imuAccelTask(void * hackflight, uint32_t time);

    void imuAccumulateGyro(hackflight_t * hf, float * adcf);

    void imuGetEulerAngles(hackflight_t * hf, timeUs_t time);

    void imuGetQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat);

    void imuInit(void);

    void imuUpdateFusion(timeUs_t time, quaternion_t * quat, rotation_t * rot);

#if defined(__cplusplus)
}
#endif
