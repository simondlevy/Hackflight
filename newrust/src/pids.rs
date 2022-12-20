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

    Angle { angpid: anglepid::Pid },

    AltHold { altpid: altholdpid::Pid },
}

pub fn make_angle(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32 ) -> PidController {

    PidController::Angle {
        angpid: anglepid::make(k_rate_p, k_rate_i, k_rate_d, k_rate_f, k_level_p)
    }
}

pub fn make_alt_hold(
    k_p: f32,
    k_i: f32) -> PidController {

    PidController::AltHold {
        altpid: altholdpid::make(k_p, k_i) 
    }
}

pub fn get_demands(
    t: &mut PidController,
    vstate: VehicleState,
    demands: Demands) -> Demands {

    match *t {

        PidController::Angle {ref mut angpid} => {
                anglepid::get_demands(angpid, &vstate, &demands)
            },

        PidController::AltHold {ref mut altpid} => {
            altholdpid::get_demands(altpid, &vstate, &demands)
        }
    }
}
