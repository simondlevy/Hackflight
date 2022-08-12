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

#include "hackflight_core.h"

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

class Task {

    private:

        uint16_t m_ageCycles;
        uint32_t m_anticipatedExecutionTime;
        int32_t  m_desiredPeriodUs;            
        uint16_t m_dynamicPriority;          
        uint32_t m_lastExecutedAtUs;          
        uint32_t m_lastSignaledAtUs;         

    public:

        Task(uint32_t rate) 
        {
            m_desiredPeriodUs = 1000000 / rate;
        }

        virtual void fun(
                hackflight_core_t * core,
                task_data_t * data,
                uint32_t usec);

}; 

typedef void (*task_fun_t)(
        hackflight_core_t * core,
        task_data_t * data,
        uint32_t usec);

typedef struct {

    task_fun_t fun;
    int32_t desiredPeriodUs;            
    uint32_t lastExecutedAtUs;          
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
