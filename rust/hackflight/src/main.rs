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

// Filtering -------------------------------------------------------------------

#[allow(dead_code)]
struct Pt1Filter {
    state: f32,
    k: f32
}

#[allow(dead_code)]
struct Pt2Filter
{
    state: f32,
    state1: f32,
    k: f32
} 

#[allow(dead_code)]
struct Pt3Filter
{
    state: f32,
    state1: f32,
    state2: f32,
    k: f32
}

#[allow(dead_code)]
struct BiquadFilter
{
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,

    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,

    weigh: f32
} 

// Angle PID control -----------------------------------------------------------

#[allow(dead_code)]
struct PidAxisData
{
    p:   f32,
    i:   f32,
    d:   f32,
    f:   f32,
    sum: f32
}

#[allow(dead_code)]
struct AnglePidAxis
{
    data : PidAxisData,
    d_min_lowpass : Pt2Filter,
    d_min_range : Pt2Filter,
    dterm_lowpass : Pt1Filter,
    dterm_lowpass2: Pt1Filter,
    windup_lpf :Pt1Filter,
    pterm_yaw_lowpass :Pt1Filter,
    previous_setpoint_correction : f32,
    previous_gyro_rate_dterm : f32,
    previous_setpoint : f32,
    feedforward_pt3: Pt3Filter,
}

#[allow(dead_code)]
struct AnglePid
{
    axis_x : AnglePidAxis,
    axis_y : AnglePidAxis,
    axis_z : AnglePidAxis,
    dyn_lpf_previous_quantized_throttle : i32,  
    feedforward_lpf_initialized : bool,
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32,
    last_dyn_lpf_update_us: u32
}

// Demands ---------------------------------------------------------------------

#[allow(dead_code)]
struct Demands 
{
    throttle: f32,
    roll: f32,
    pitch: f32,
    yaw: f32
} 

// Axes ------------------------------------------------------------------------

#[allow(dead_code)]
struct Axes 
{
    x: f32,
    y: f32,
    z: f32
}

// Vehicle state ---------------------------------------------------------------

#[allow(dead_code)]
struct VehicleState 
{
    x: f32,
    dx: f32,
    y:  f32,
    dy: f32,
    z: f32,
    dz: f32,
    phi: f32,
    dphi: f32,
    theta: f32,
    dtheta: f32,
    psi: f32,
    dpsi: f32
}

// General PID control ---------------------------------------------------------
// XXX

// Tasks -----------------------------------------------------------------------
// XXX


// IMU -------------------------------------------------------------------------

#[allow(dead_code)]
struct Quaternion
{
    w: f32,
    x: f32,
    y: f32,
    z: f32
} 

#[allow(dead_code)]
struct Rotation
{
    r20: f32,
    r21: f32,
    r22: f32
}

#[allow(dead_code)]
struct ImuSensor
{
    values: Axes,
    count: u32
}

#[allow(dead_code)]
struct GyroReset
{
    quiet_period_end: u32,
    reset_time_end: u32,
    reset_completed: bool
}

#[allow(dead_code)]
struct ImuFusion
{
    time: u32,
    quat: Quaternion,
    rot: Rotation,
    gyro_reset: GyroReset
}


// Stats -----------------------------------------------------------------------

#[allow(dead_code)]
struct StdDev
{
    m_old_m: f32,
    m_new_m: f32,
    m_old_s: f32,
    m_new_s: f32,
    m_n : i32 // XXX should be u32 ?
}


// Filters ---------------------------------------------------------------------
// XXX

// Gyro ------------------------------------------------------------------------

#[allow(dead_code)]
struct CalibrationAxis
{
    sum: f32,
    var: StdDev
}

#[allow(dead_code)]
struct GyroCalibration
{
    x: CalibrationAxis,
    y: CalibrationAxis,
    z: CalibrationAxis,

    cycles_remaining: i32

}

/* XXX
union {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;
*/

#[allow(dead_code)]
struct GyroAxis
{
    dps : f32,          // aligned, calibrated, scaled, unfiltered
    dps_filtered : f32, // filtered
    sample_count : u8,  // gyro sensor sample counter
    sample_sum : f32,   // summed samples used for downsampling
    zero: f32,
    //gyroLowpassFilter_t lowpassFilter : GyroLowpassFilter,
    //gyroLowpassFilter_t lowpass2Filter : GyroLowpassFilter,
}

#[allow(dead_code)]
struct Gyro 
{
    x : GyroAxis,
    y : GyroAxis,
    z : GyroAxis,

    accum : ImuSensor,

    is_calibrating : bool,

    // if true then downsample using gyro lowpass 2, otherwise use averaging
    downsample_filter_enabled : bool,      

    calibration: GyroCalibration,

    // lowpass gyro soft filter
    // lowpass_filter_apply_fn : FilterApplyFn,

    // lowpass2 gyro soft filter
    // lowpass2_filter_apply_fn : FilterApplyFn
}

// Serial ports ----------------------------------------------------------------

#[allow(dead_code)]
enum SerialPortIdentifier
{
    SerialPortAll = -2,
    SerialPortNone = -1,
    SerialPortUsart1 = 0,
    SerialPortUsart2,
    SerialPortUsart3,
    SerialPortUsart4,
    SerialPortUsart5,
    SerialPortUsart6,
    SerialPortUsart7,
    SerialPortUsart8,
    SerialPortUsart9,
    SerialPortUsart10,
    SerialPortUsbVcp = 20,
    SerialPortSoftSerial1 = 30,
    SerialPortSoftSerial2,
    SerialPortLpUart1 = 40,
}

// Arming ---------------------------------------------------------------------

#[allow(dead_code)]
struct Arming
{
    acc_done_calibrating: bool,
    angle_okay: bool,
    arming_switch_okay: bool,
    gyro_done_calibrating: bool,
    dshot_bitbang_okay: bool,
    is_armed: bool,
    rx_failsafe_okay: bool,
    throttle_is_down: bool
}

// Receiver --------------------------------------------------------------------

#[allow(dead_code)]
struct RxAxes
{
    demands : Demands,
    aux1 : f32,
    aux2 : f32
}

#[allow(dead_code)]
enum RxFrameState
{
    RxFramePending = 0,
    RxFrameComplete = (1 << 0),
    RxFrameFailsafe = (1 << 1),
    RxFrameProcessingRequired = (1 << 2),
    RxFrameDropped = (1 << 3)
}

#[allow(dead_code)]
enum RxFailsafeChannelMode
{
    RxFailsafeModeAuto,
    RxFailsafeModeHold,
    RxFailsafeModeSet,
    RxFailsafeModeInvalid
} 

/*
#[allow(dead_code)]
enum 
{
    MODELOGIC_OR = 0,
    MODELOGIC_AND
} modeLogic_e;

#[allow(dead_code)]
enum 
{
    // ARM flag
    BOXARM = 0,
    CHECKBOX_ITEM_COUNT
} boxId_e;

#[allow(dead_code)]
struct rxFailsafeChannelConfig_s 
{
    uint8_t mode; 
    uint8_t step;
}

#[allow(dead_code)]
struct rxChannelRangeConfig_s 
{
    uint16_t min;
    uint16_t max;
}

#[allow(dead_code)]
enum 
{
    RX_STATE_CHECK,
    RX_STATE_PROCESS,
    RX_STATE_MODES,
    RX_STATE_UPDATE,
    RX_STATE_COUNT
} rxState_e;

#[allow(dead_code)]
struct rxSmoothingFilter_s 
{
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

}
*/

// Scheduling ------------------------------------------------------------------

#[allow(dead_code)]
struct Scheduler
{
    loop_start_cycles: i32,
    loop_start_min_cycles: i32,
    loop_start_max_cycles: i32,
    loop_start_delta_down_cycles: u32,
    loop_start_delta_up_cycles: u32,

    task_guard_cycles: i32,
    task_guard_min_cycles: i32,
    task_guard_max_cycles: i32,
    task_guard_delta_down_cycles: u32,
    task_guard_delta_up_cycles: u32,

    desired_period_cycles: i32,
    last_target_cycles: u32,

    next_timing_cycles: u32,

    guard_margin: i32,
    clock_rate: u32
}


fn main() {
    println!("Hello, world!");
}
