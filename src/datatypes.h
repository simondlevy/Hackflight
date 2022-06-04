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
        uint32_t usec
        , demands_t * demands
        , void * data
        , vehicle_state_t * vstate
        , bool reset
        );

typedef struct {
    pid_fun_t fun;
    void * data;
} pid_controller_t;
