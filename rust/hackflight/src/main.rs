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

// Filtering ------------------------------------------------------------------

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

// PID control ----------------------------------------------------------------

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
    // pidAxisData_t  data[3],
    // pt2Filter_t    dMinLowpass[3],
    // pt2Filter_t    dMinRange[3],
    // pt1Filter_t    dtermLowpass[3],
    // pt1Filter_t    dtermLowpass2[3],
    // pt1Filter_t    windupLpf[3],
    // pt1Filter_t    ptermYawLowpass,
    // float          previousSetpointCorrection[3],
    // float          previousGyroRateDterm[3],
    // float          previousSetpoint[3],
    // pt3Filter_t    feedforwardPt3[3],
    dyn_lpf_previous_quantized_throttle : i32,  
    feedforward_lpf_initialized : bool,
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32,
    last_dyn_lpf_update_us: u32
}

/// Vehicle state ------------------------------------------------------------------------

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
