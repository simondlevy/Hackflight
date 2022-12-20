/*
   Hackflight altitude-hold PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::VehicleState;

const ALTITUDE_MIN: f32   = 1.0;
const PILOT_VELZ_MAX: f32 = 2.5;
const STICK_DEADBAND: f32 = 0.2;
const WINDUP_MAX: f32     = 0.4;

#[derive(Debug,Clone)]
pub struct Pid { 
    k_p : f32,
    k_i: f32, 
    in_band_prev: bool,
}

pub fn make(
    k_p: f32,
    k_i: f32) -> Pid {

    Pid {
        k_p: k_p,
        k_i: k_i,
        in_band_prev: false
    }
} 

pub fn get_demands(
    pid: &mut Pid,
    demands: &Demands,
    vstate: &VehicleState,
    reset: &bool) -> Demands {

    pid.in_band_prev = false;

    Demands {throttle: 0.0, roll:0.0, pitch: 0.0, yaw: 0.0}
}
