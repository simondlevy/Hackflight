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

// Demands ----------------------------------------------------------------------

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} demands_t;

// Axes ------------------------------------------------------------------------

typedef struct {
    float x;
    float y;
    float z;
} axes_t;

// Vehicle state ----------------------------------------------------------------

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

// General PID control ---------------------------------------------------------

typedef void (*pid_fun_t)(
        uint32_t usec,
        demands_t * demands,
        void * data,
        vehicle_state_t * vstate,
        bool reset
        );


typedef struct {
    pid_fun_t fun;
    void * data;
} pid_controller_t;


// IMU ------------------------------------------------------------------------

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
    axes_t values;
    uint32_t count;
} imu_sensor_t;

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

typedef void (*imu_align_fun)(axes_t * axes);

 // Stats ------------------------------------------------------------------------

typedef struct {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n; // XXX should be uint32_t ?
} stdev_t;

// Stick indices --------------------------------------------------------------

typedef enum rc_alias {
    THROTTLE,
    ROLL,
    PITCH,
    YAW,
    AUX1,
    AUX2
} rc_alias_e;
