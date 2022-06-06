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

#include "time.h"

// Filtering ------------------------------------------------------------------

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef struct {
    float state;
    float state1;
    float k;
} pt2Filter_t;

typedef struct {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

// PID control ----------------------------------------------------------------

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    biquadFilter_t biquadFilter;
    pt1Filter_t    pt1Filter;
    pt2Filter_t    pt2Filter;
    pt3Filter_t    pt3Filter;
} dtermLowpass_t;

typedef struct {
    pidAxisData_t  data[3];
    pt2Filter_t    dMinLowpass[3];
    pt2Filter_t    dMinRange[3];
    dtermLowpass_t dtermLowpass[3];
    dtermLowpass_t dtermLowpass2[3];
    int32_t        dynLpfPreviousQuantizedThrottle;  
    bool           feedforwardLpfInitialized;
    pt3Filter_t    feedforwardPt3[3];
    float          k_rate_p;
    float          k_rate_i;
    float          k_rate_d;
    float          k_rate_f;
    float          k_level_p;
    timeUs_t       lastDynLpfUpdateUs;
    float          previousSetpointCorrection[3];
    float          previousGyroRateDterm[3];
    float          previousSetpoint[3];
    pt1Filter_t    ptermYawLowpass;
    pt1Filter_t    windupLpf[3];

} angle_pid_t;
// Time -----------------------------------------------------------------------

typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;

// Demands ------------------------------------------------------------------------

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

// Receiver ------------------------------------------------------------------------

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

// Vehicle state ------------------------------------------------------------------------

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

// PID control ------------------------------------------------------------------------

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


// Tasks ------------------------------------------------------------------------

typedef void (*task_fun_t)(
        void * hackflight,
        uint32_t usec
        );

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

// Stats ------------------------------------------------------------------------

typedef struct {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

// Filters ----------------------------------------------------------------------

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

struct filter_s;
typedef struct filter_s filter_t;
typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

// Gyro ------------------------------------------------------------------------

typedef struct {
    float sum[3];
    stdev_t var[3];
    int32_t cyclesRemaining;
} gyroCalibration_t;

// Hackflight ------------------------------------------------------------------

typedef struct {

    imu_sensor_t     accelAccum;
    angle_pid_t      anglepid;
    bool             armed;
    task_t           attitudeTask;
    demands_t        demands;
    imu_sensor_t     gyroAccum;
    bool             gyroIsCalibrating;
    imu_fusion_t     imuFusionPrev;
    float            mspmotors[4];
    pid_controller_t pid_controllers[10];
    uint8_t          pid_count;
    bool             pid_zero_throttle_iterm_reset;
    task_t           rxTask;
    rx_axes_t        rx_axes;
    task_t           sensor_tasks[20];
    uint8_t          sensor_task_count;
    vehicle_state_t  vstate;

} hackflight_t;

