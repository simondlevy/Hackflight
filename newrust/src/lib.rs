/*
   Hackflight Rust library

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

pub mod pids;
pub mod utils;
pub mod filters;
pub mod mixers;

pub mod anglepid;
pub mod altholdpid;

#[derive(Clone)]
pub struct Demands {
    pub throttle: f32,
    pub roll:     f32,
    pub pitch:    f32,
    pub yaw:      f32
} 

#[derive(Clone,Copy)]
pub struct VehicleState {

    pub x:      f32,
    pub dx:     f32,
    pub y:      f32,
    pub dy:     f32,
    pub z:      f32,
    pub dz:     f32,
    pub phi:    f32,
    pub dphi:   f32,
    pub theta:  f32,
    pub dtheta: f32,
    pub psi:    f32,
    pub dpsi:   f32
}

pub struct Motors {

    pub m1: f32,
    pub m2: f32,
    pub m3: f32,
    pub m4: f32
}

pub fn run_pids(
    arr: &mut [pids::Controller],
    vstate: &VehicleState,
    demands: &Demands) -> Demands {

    let mut new_demands = Demands {
        throttle: demands.throttle, 
        roll: demands.roll,
        pitch: demands.pitch,
        yaw: demands.yaw
    };

    for pid in arr.iter_mut() {
        new_demands = pids::get_demands(&mut *pid, *vstate, new_demands);
    }

    new_demands
}


