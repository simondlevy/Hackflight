/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

#[derive(Debug,Clone)]
pub struct AnglePid { 
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

pub fn make_angle_pid(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32) -> AnglePid {

    AnglePid {
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

pub fn angpid_get_demands(angpid: &mut AnglePid) -> Demands {

    angpid.dyn_lpf_previous_quantized_throttle = 0;

    Demands {throttle: 0.0, roll:0.0, pitch: 0.0, yaw: 0.0}
}

/////////////////////////////////////////////////////////////////////////////////

#[derive(Debug,Clone)]
pub struct AltHoldPid { 
    k_p : f32,
    k_i: f32, 
    in_band_prev: bool,
}

pub fn make_alt_hold_pid(
    k_p: f32,
    k_i: f32) -> AltHoldPid {

    AltHoldPid {
        k_p: k_p,
        k_i: k_i,
        in_band_prev: false
    }
} 

pub fn altpid_get_demands(altpid: &mut AltHoldPid) -> Demands {

    altpid.in_band_prev = false;

    Demands {throttle: 0.0, roll:0.0, pitch: 0.0, yaw: 0.0}
}

/////////////////////////////////////////////////////////////////////////////////

#[derive(Debug,Clone)]
pub enum PidController {

    Angle { angpid: AnglePid },

    AltHold { altpid: AltHoldPid },
}

pub fn make_angle(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32 ) -> PidController {

    PidController::Angle {
        angpid: make_angle_pid(k_rate_p, k_rate_i, k_rate_d, k_rate_f, k_level_p)
    }
}

pub fn make_alt_hold(
    k_p: f32,
    k_i: f32) -> PidController {

    PidController::AltHold {
        altpid: make_alt_hold_pid(k_p, k_i) 
    }
}

pub fn get_demands(t: &mut PidController, vstate: VehicleState) -> Demands {

    match *t {

        PidController::Angle {ref mut angpid} => {angpid_get_demands(angpid)},

        PidController::AltHold {ref mut altpid} => {altpid_get_demands(altpid)}
    }
}
