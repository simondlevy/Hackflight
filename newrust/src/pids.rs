/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

mod angle;
mod althold;

#[derive(Debug,Clone)]
pub enum Controller {

    Angle { angpid: angle::Pid },

    AltHold { altpid: althold::Pid },
}

pub fn make_angle(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32 ) -> Controller {

    Controller::Angle {
        angpid: angle::make(k_rate_p, k_rate_i, k_rate_d, k_rate_f, k_level_p)
    }
}

pub fn make_alt_hold(
    k_p: f32,
    k_i: f32) -> Controller {

    Controller::AltHold {
        altpid: althold::make(k_p, k_i) 
    }
}

pub fn get_demands(
    t: &mut Controller,
    demands: Demands,
    vstate: VehicleState,
    reset: bool) -> Demands {

    match *t {

        Controller::Angle {ref mut angpid} => {
                angle::get_demands(angpid, &demands, &vstate, &reset)
            },

        Controller::AltHold {ref mut altpid} => {
            althold::get_demands(altpid, &demands, &vstate, &reset)
        }
    }
}
