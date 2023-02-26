/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

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

use crate::Demands;
use crate::VehicleState;

mod angle;
mod althold;

#[derive(Clone)]
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

pub fn update(
    t: &mut Controller,
    usec: u32,
    demands: Demands,
    vstate: VehicleState,
    pid_reset: bool) -> Demands {

    match *t {

        Controller::Angle {ref mut angpid} => {
                angle::get_demands(angpid, &usec, &demands, &vstate, &pid_reset)
            },

        Controller::AltHold {ref mut altpid} => {
            althold::get_demands(altpid, &demands, &vstate, &pid_reset)
        }
    }
}
