/*
   Hackflight altitude-hold PID controller support

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
use crate::utils;

const ALTITUDE_MIN: f32   = 1.0;
const PILOT_VELZ_MAX: f32 = 2.5;
const STICK_DEADBAND: f32 = 0.2;
const WINDUP_MAX: f32     = 0.4;

#[derive(Debug,Clone)]
pub struct Pid { 
    k_p : f32,
    k_i: f32, 
    in_band_prev: bool,
    error_integral: f32,
    altitude_target: f32
}

pub fn make(
    k_p: f32,
    k_i: f32) -> Pid {

    Pid {
        k_p: k_p,
        k_i: k_i,
        in_band_prev: false,
        error_integral: 0.0,
        altitude_target: 0.0
    }
} 

pub fn get_demands(
    pid: &mut Pid,
    demands: &Demands,
    vstate: &VehicleState,
    reset: &bool) -> Demands {

    let altitude = vstate.z;
    let dz = vstate.dz;

    // [0,1] => [-1,+1]
    let sthrottle = 2.0 * demands.throttle - 1.0; 

    // Is stick demand in deadband, above a minimum altitude?
    let in_band = sthrottle.abs() < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

    // Reset controller when moving into deadband above a minimum altitude
    let got_new_target = in_band && !pid.in_band_prev;
    pid.error_integral = if got_new_target || *reset { 0.0 } else { pid.error_integral };

    pid.in_band_prev = in_band;

    pid.altitude_target = if *reset { 0.0 } else { pid.altitude_target }; 

    pid.altitude_target = if got_new_target { altitude } else { pid.altitude_target };

    // Target velocity is a setpoint inside deadband, scaled constant outside
    let target_velocity =
        if in_band {pid.altitude_target - altitude } else { PILOT_VELZ_MAX * sthrottle};

    // Compute error as scaled target minus actual
    let error = target_velocity - dz;

    // Compute I term, avoiding windup
    pid.error_integral = utils::constrain_abs(pid.error_integral + error, WINDUP_MAX);

    // Adjust throttle demand based on error
    Demands { 
        throttle : demands.throttle + (error * pid.k_p + pid.error_integral * pid.k_i),
        roll : demands.roll,
        pitch : demands.pitch,
        yaw : demands.yaw
    }
}









