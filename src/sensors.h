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

#include "init_task.h"

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
} sensors_e;

#if defined(__cplusplus)
extern "C" {
#endif

static void hackflightAddSensor(hackflight_t * hf, task_fun_t fun, uint32_t rate)
{
    initTask(&hf->sensorTasks[hf->sensorTaskCount++], fun, rate);
}

#if defined(__cplusplus)
}
#endif
