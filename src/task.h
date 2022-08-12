/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {

    arming_t         arming;
    gyro_t           gyro;
    imu_fusion_t     imuFusionPrev;
    float            maxArmingAngle;
    void *           motorDevice;
    float            mspMotors[4];
    rx_t             rx;
    rx_axes_t        rxAxes;

} task_data_t;

typedef void (*task_fun_t)(void * hp, void * dp, uint32_t usec);

typedef struct {

    // For both hardware and sim implementations
    void (*fun)(void * hp, void * dp, uint32_t time);
    int32_t desiredPeriodUs;            
    uint32_t lastExecutedAtUs;          

    // For hardware impelmentations
    uint16_t dynamicPriority;          
    uint16_t taskAgeCycles;
    uint32_t lastSignaledAtUs;         
    uint32_t anticipatedExecutionTime;

} task_t;


static void initTask(task_t * task, task_fun_t fun, uint32_t rate)
{
    task->fun = fun;
    task->desiredPeriodUs = 1000000 / rate;
}



