/*
   Hackflight Rust library

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

pub mod pids;
pub mod mixers;
pub mod filters;
pub mod utils;

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

pub trait Mixer {

    fn get_motors(&self, demands: & Demands) -> Motors;
}

// Corresponds to C++ Mixer::step()
pub fn step(
    stick_demands: &Demands,
    state: &VehicleState,
    arr: &mut [pids::Controller],
    pid_reset: &bool,
    usec: & u32,
    mixer: &dyn Mixer) -> Motors {

        let mut demands = stick_demands.clone();

        for pid in arr.iter_mut() {
            demands = pids::update(&mut *pid, *usec, demands, *state, *pid_reset);
        }

        mixer.get_motors(&demands)
}


