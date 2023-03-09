/*
   Copyright (c) 2023 Simon D. Levy

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

#![no_std]

use panic_halt as _; 

pub mod anglepid;
pub mod datatypes;
pub mod filters;
pub mod mixers;
pub mod utils;

use crate::datatypes::Demands;
// use crate::datatypes::VehicleState;
use crate::mixers::quadxbf_mix;

static mut ANGLE_PID : anglepid::AnglePid = anglepid::INIT;

#[no_mangle]
pub extern "C" fn rust_init() {

    unsafe {
        ANGLE_PID.init();
    }
}

#[no_mangle]
pub extern "C" fn rust_get_motors(
    _usec: u32,
    thr: f32, rol: f32, pit: f32, yaw: f32,
    _angvel_x: f32, _angvel_y: f32, _angvel_z: f32,
    _reset: bool,
    m1: *mut f32, m2: *mut f32, m3: *mut f32, m4: *mut f32) {

    let demands = Demands{
        throttle: thr,
        roll: rol,
        pitch: pit,
        yaw: yaw
    };

    /*
    let vstate = VehicleState {
        x: 0.,
        dx: 0.,
        y: 0.,
        dy: 0.,
        z: 0.,
        dz: 0.,
        phi: 0.,
        dphi: angvel_x,
        theta: 0.,
        dtheta: angvel_y,
        psi: 0.,
        dpsi: angvel_z,
    };*/

    unsafe {

        // let new_demands = ANGLE_PID.get_demands(usec, demands, vstate, reset);

        (*m1, *m2, *m3, *m4) = quadxbf_mix(demands);
    }
}
