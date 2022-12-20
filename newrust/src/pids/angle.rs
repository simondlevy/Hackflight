/*
   Hackflight angle PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

#[derive(Debug,Clone)]
pub struct Pid { 
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32,
    //roll : CyclicAxis,
    //pitch : CyclicAxis,
    //yaw: Axis,
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
        //roll : CyclicAxis,
        //pitch : CyclicAxis,
        //yaw: Axis,
        dyn_lpf_previous_quantized_throttle: 0, 
    }
} 

pub fn get_demands(
    pid: &mut Pid,
    demands: &Demands,
    vstate: &VehicleState,
    reset: &bool) -> Demands {

    pid.dyn_lpf_previous_quantized_throttle = 0;

    Demands {throttle: 0.0, roll:0.0, pitch: 0.0, yaw: 0.0}
}
