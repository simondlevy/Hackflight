/*
   Hackflight angle PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

use crate::filters;
use crate::utils::DT;
use crate::utils::constrain_f;

const DTERM_LPF1_DYN_MIN_HZ: f32 = 75.0;
const DTERM_LPF1_DYN_MAX_HZ: f32 = 150.0;
const DTERM_LPF2_HZ: f32 = 150.0;
const D_MIN_LOWPASS_HZ: f32 = 35.0;  
const ITERM_RELAX_CUTOFF: f32 = 15.0;
const D_MIN_RANGE_HZ: f32 = 85.0;  

// minimum of 5ms between updates
const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u32 = 5000; 

const DYN_LPF_THROTTLE_STEPS: f32 = 100.0;

const ITERM_LIMIT: f32 = 400.0;

const ITERM_WINDUP_POINT_PERCENT: f32 = 85.0;        

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
const ITERM_RELAX_SETPOINT_THRESHOLD: f32 = 40.0;

const D_MIN: f32 = 30.0;
const D_MIN_GAIN: f32 = 37.0;
const D_MIN_ADVANCE: f32 = 20.0;

const FEEDFORWARD_MAX_RATE_LIMIT: f32 = 900.0;

const DYN_LPF_CURVE_EXPO: f32 = 5.0;

// PT2 lowpass cutoff to smooth the boost effect
const D_MIN_GAIN_FACTOR: f32  = 0.00008;
const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;

const RATE_ACCEL_LIMIT: f32 = 0.0;
const YAW_RATE_ACCEL_LIMIT: f32 = 0.0;

const OUTPUT_SCALING: f32 = 1000.0;
const  LIMIT_CYCLIC: f32 = 500.0; 
const  LIMIT_YAW: f32 = 400.0;

#[derive(Clone)]
pub struct Pid { 
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32,
    roll : CyclicAxis,
    pitch : CyclicAxis,
    yaw: Axis,
    dyn_lpf_previous_quantized_throttle: i32,  
    //pterm_yaw_lpf: filters::Pt1
}

pub fn make(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32) -> Pid {

        Pid {
            k_rate_p: k_rate_p,
            k_rate_i: k_rate_i,
            k_rate_d: k_rate_d,
            k_rate_f: k_rate_f,
            k_level_p: k_level_p,
            roll : make_cyclic_axis(),
            pitch : make_cyclic_axis(),
            yaw: make_axis(),
            dyn_lpf_previous_quantized_throttle: 0, 
        }
} 

pub fn get_demands(
    pid: &mut Pid,
    demands: &Demands,
    vstate: &VehicleState,
    reset: &bool) -> Demands {

        let roll_demand  = rescale(demands.roll);
        let pitch_demand = rescale(demands.pitch);
        let yaw_demand   = rescale(demands.yaw);

        Demands {throttle: 0.0, roll:0.0, pitch: 0.0, yaw: 0.0}
}

#[derive(Clone,Copy)]
struct Axis {

    previous_setpoint : f32,
    integral : f32
}

#[derive(Clone)]
struct CyclicAxis {

    axis: Axis,
    dterm_lpf1 : filters::Pt1,
    dterm_lpf2 : filters::Pt1,
    d_min_lpf: filters::Pt2,
    d_min_range: filters::Pt2,
    windup_lpf: filters::Pt1,
    previous_dterm: f32
}

fn make_cyclic_axis() -> CyclicAxis {

    CyclicAxis {
        axis: make_axis(),
        dterm_lpf1 : filters::make_pt1(DTERM_LPF1_DYN_MIN_HZ),
        dterm_lpf2 : filters::make_pt1(DTERM_LPF2_HZ),
        d_min_lpf: filters::make_pt2(D_MIN_LOWPASS_HZ),
        d_min_range: filters::make_pt2(D_MIN_RANGE_HZ),
        windup_lpf: filters::make_pt1(ITERM_RELAX_CUTOFF),
        previous_dterm: 0.0 }
}

fn make_axis() -> Axis {

    Axis { previous_setpoint: 0.0, integral: 0.0 }
}

// [-1,+1] => [-670,+670] with nonlinearity
fn rescale(command: f32) -> f32 {

    const CTR: f32 = 0.104;

    let expof = command * command.abs();
    let angle_rate = command * CTR + (1.0 - CTR) * expof;

    670.0 * angle_rate
}


