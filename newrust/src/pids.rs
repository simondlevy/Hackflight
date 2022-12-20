/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

use crate::anglepid;
use crate::altholdpid;

#[derive(Debug,Clone)]
pub enum PidController {

    Angle { angpid: anglepid::AnglePid },

    AltHold { altpid: altholdpid::AltHoldPid },
}

pub fn make_angle(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32 ) -> PidController {

    PidController::Angle {
        angpid: anglepid::make_angle_pid(k_rate_p, k_rate_i, k_rate_d, k_rate_f, k_level_p)
    }
}

pub fn make_alt_hold(
    k_p: f32,
    k_i: f32) -> PidController {

    PidController::AltHold {
        altpid: altholdpid::make_alt_hold_pid(k_p, k_i) 
    }
}

pub fn get_demands(t: &mut PidController, vstate: VehicleState) -> Demands {

    match *t {

        PidController::Angle {ref mut angpid} => {anglepid::angpid_get_demands(angpid)},

        PidController::AltHold {ref mut altpid} => {altholdpid::altpid_get_demands(altpid)}
    }
}
