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

#include "pids/angle_struct.h"

typedef uint32_t timeUs_t;

typedef int32_t timeDelta_t;

typedef struct {

    float w;
    float x;
    float y;
    float z;

} quaternion_t;

typedef struct {

    float r20;
    float r21;
    float r22;

} rotation_t;

typedef struct {

    float throttle;
    float roll;
    float pitch;
    float yaw;

} demands_t;

typedef struct {

    float x;
    float y;
    float z;

} axes_t;

typedef struct {

    demands_t demands;
    float aux1;
    float aux2;

} rx_axes_t;

typedef enum rc_alias {
    THROTTLE,
    ROLL,
    PITCH,
    YAW,
    AUX1,
    AUX2
} rc_alias_e;

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} rxFrameState_e;

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef struct {

    float x;
    float dx;
    float y;
    float dy;
    float z;
    float dz;
    float phi;
    float dphi;
    float theta;
    float dtheta;
    float psi;
    float dpsi;

} vehicle_state_t;

typedef void (*pid_fun_t)(
        uint32_t usec,
        demands_t * demands,
        void * data,
        vehicle_state_t * vstate,
        bool reset
        );


typedef void (*task_fun_t)(
        void * hackflight,
        uint32_t usec
        );

typedef struct {
    pid_fun_t fun;
    void * data;
} pid_controller_t;


typedef struct {

    // For both hardware and sim implementations
    void (*fun)(void * hackflight, uint32_t time);
    timeDelta_t desiredPeriodUs;        // target period of execution
    timeUs_t lastExecutedAtUs;          // last time of invocation

    // For hardware impelmentations
    uint16_t dynamicPriority;           // when last executed, to avoid task starvation
    uint16_t taskAgeCycles;
    timeUs_t lastSignaledAtUs;          // time of invocation event for event-driven tasks
    timeUs_t anticipatedExecutionTime;  // Fixed point expectation of next execution time

} task_t;

typedef struct {

    angle_pid_t      anglepid;
    bool             armed;
    task_t           attitudeTask;
    demands_t        demands;
    bool             gyro_is_calibrating;
    float            mspmotors[4];
    pid_controller_t pid_controllers[10];
    uint8_t          pid_count;
    bool             pid_zero_throttle_iterm_reset;
    task_t           rxTask;
    rx_axes_t        rx_axes;
    task_t           sensor_tasks[20];
    uint8_t          sensor_task_count;
    vehicle_state_t  state;

} hackflight_t;
