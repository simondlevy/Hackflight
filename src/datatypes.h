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

// Angle PID control ----------------------------------------------------------

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
    uint32_t       lastDynLpfUpdateUs;
    float          previousSetpointCorrection[3];
    float          previousGyroRateDterm[3];
    float          previousSetpoint[3];
    pt1Filter_t    ptermYawLowpass;
    pt1Filter_t    windupLpf[3];
} angle_pid_t;

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


// Tasks ------------------------------------------------------------------------

typedef void (*task_fun_t)(
        void * hackflight,
        uint32_t usec
        );

typedef struct {

    // For both hardware and sim implementations
    void (*fun)(void * hackflight, uint32_t time);
    int32_t desiredPeriodUs;            
    uint32_t lastExecutedAtUs;          

    // For hardware impelmentations
    uint16_t dynamicPriority;          
    uint16_t taskAgeCycles;
    uint32_t lastSignaledAtUs;         
    uint32_t anticipatedExecutionTime;

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

typedef void (*imu_align_fun)(axes_t * axes);

 // Stats ------------------------------------------------------------------------

typedef struct {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n; // XXX should be uint32_t ?
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

typedef union {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef struct {
    imu_sensor_t accum;
    float        dps[3];          // aligned, calibrated, scaled, unfiltered
    float        dps_filtered[3]; // filtered 
    uint8_t      sampleCount;     // sample counter
    float        sampleSum[3];    // summed samples used for downsampling
    bool         isCalibrating;

    // if true then downsample using gyro lowpass 2, otherwise use averaging
    bool downsampleFilterEnabled;      

    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[3];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[3];

    float zero[3];
} gyro_t;

// Serial ports ----------------------------------------------------------------

typedef enum {
    SERIAL_PORT_ALL = -2,
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_UART9,
    SERIAL_PORT_USART10,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_LPUART1 = 40,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_LPUART1,
} serialPortIdentifier_e;

// Scheduling ------------------------------------------------------------------

typedef struct {
    int32_t loopStartCycles;
    int32_t loopStartMinCycles;
    int32_t loopStartMaxCycles;
    uint32_t loopStartDeltaDownCycles;
    uint32_t loopStartDeltaUpCycles;

    int32_t taskGuardCycles;
    int32_t taskGuardMinCycles;
    int32_t taskGuardMaxCycles;
    uint32_t taskGuardDeltaDownCycles;
    uint32_t taskGuardDeltaUpCycles;

    int32_t desiredPeriodCycles;
    uint32_t lastTargetCycles;

    uint32_t nextTimingCycles;

    int32_t guardMargin;
    uint32_t clockRate;

} scheduler_t;

// Arming ---------------------------------------------------------------------

typedef struct {

    bool acc_done_calibrating;
    bool angle_okay;
    bool arming_switch_okay;
    bool gyro_done_calibrating;
    bool dshot_bitbang_okay;
    bool is_armed;
    bool rx_failsafe_okay;
    bool throttle_is_down;

} arming_t;


// Receiver --------------------------------------------------------------------

#define CHANNEL_COUNT 18
#define THROTTLE_LOOKUP_LENGTH 12

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
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID
} rxFailsafeChannelMode_e;

typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode; 
    uint8_t step;
} rxFailsafeChannelConfig_t;

typedef struct rxChannelRangeConfig_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfig_t;


typedef enum {
    RX_STATE_CHECK,
    RX_STATE_PROCESS,
    RX_STATE_MODES,
    RX_STATE_UPDATE,
    RX_STATE_COUNT
} rxState_e;

typedef struct rxSmoothingFilter_s {

    uint8_t     autoSmoothnessFactorSetpoint;
    uint32_t    averageFrameTimeUs;
    uint8_t     autoSmoothnessFactorThrottle;
    uint16_t    feedforwardCutoffFrequency;
    uint8_t     ffCutoffSetting;

    pt3Filter_t filterThrottle;
    pt3Filter_t filterRoll;
    pt3Filter_t filterPitch;
    pt3Filter_t filterYaw;

    pt3Filter_t filterDeflectionRoll;
    pt3Filter_t filterDeflectionPitch;

    bool        filterInitialized;
    uint16_t    setpointCutoffFrequency;
    uint8_t     setpointCutoffSetting;
    uint16_t    throttleCutoffFrequency;
    uint8_t     throttleCutoffSetting;
    float       trainingSum;
    uint32_t    trainingCount;
    uint16_t    trainingMax;
    uint16_t    trainingMin;

} rxSmoothingFilter_t;

typedef struct {

    rxSmoothingFilter_t smoothingFilter;

    bool        auxiliaryProcessingRequired;
    bool        calculatedCutoffs;
    uint16_t    channelData[CHANNEL_COUNT];
    float       command[4];
    demands_t   commands;
    bool        dataProcessingRequired;
    demands_t   dataToSmooth;
    int32_t     frameTimeDeltaUs;
    bool        gotNewData;
    bool        inFailsafeMode;
    bool        initializedFilter;
    bool        initializedThrottleTable;
    uint32_t    invalidPulsePeriod[CHANNEL_COUNT];
    bool        isRateValid;
    uint32_t    lastFrameTimeUs;
    uint32_t    lastRxTimeUs;
    int16_t     lookupThrottleRc[THROTTLE_LOOKUP_LENGTH];
    uint32_t    needSignalBefore;
    uint32_t    nextUpdateAtUs;
    uint32_t    previousFrameTimeUs;
    float       raw[CHANNEL_COUNT];
    uint32_t    refreshPeriod;
    bool        signalReceived;
    rxState_e   state;
    uint32_t    validFrameTimeMs;

} rx_t;


// Mixer -----------------------------------------------------------------------

typedef void (*mixer_t)(float throttle, float roll, float pitch, float yaw,
        float * motors);

// Hackflight ------------------------------------------------------------------

typedef struct {

    imu_sensor_t     accelAccum;
    arming_t         arming;
    angle_pid_t      anglepid;
    task_t           attitudeTask;
    demands_t        demands;
    gyro_t           gyro;
    imu_align_fun    imuAlignFun;
    imu_fusion_t     imuFusionPrev;
    float            maxArmingAngle;
    mixer_t          mixer;
    void *           motorDevice;
    float            mspMotors[4];
    task_t           mspTask;
    pid_controller_t pidControllers[10];
    uint8_t          pidCount;
    bool             pidZeroThrottleItermReset;
    rx_t             rx;
    task_t           rxTask;
    rx_axes_t        rxAxes;
    scheduler_t      scheduler;
    task_t           sensorTasks[10];
    uint8_t          sensorTaskCount;
    vehicle_state_t  vstate;

} hackflight_t;

