/*
   Hackflight Rust library

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

pub mod pids;
pub mod mixers;

mod utils;

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

pub struct QuadXbf {

}

impl Mixer for QuadXbf {

    /*
       Mixer values for quad-X frames using Betaflight motor layout:

       4cw   2ccw
       \ /
       ^
       / \
       3ccw  1cw

       Copyright (C) 2022 Simon D. Levy

       MIT License
     */
    fn get_motors(&self, demands: &Demands) -> Motors {


        Motors {

            // right rear
            m1: demands.throttle - demands.roll + demands.pitch + demands.yaw, 

            // right front
            m2: demands.throttle - demands.roll - demands.pitch - demands.yaw, 

            // left rear
            m3: demands.throttle + demands.roll + demands.pitch - demands.yaw, 

            // left front
            m4: demands.throttle + demands.roll - demands.pitch + demands.yaw  
        }
    }
}

pub fn run(
    arr: &mut [pids::Controller],
    vstate: &VehicleState,
    rxdemands: &Demands,
    mixer: &dyn Mixer) -> Motors{

    Motors { m1: 0.0, m2: 0.0, m3: 0.0, m4: 0.0 }

}

pub fn run_pids(
    arr: &mut [pids::Controller],
    vstate: &VehicleState,
    rxdemands: &Demands) -> Demands {

    let mut demands = rxdemands.clone();

    for pid in arr.iter_mut() {
        demands = pids::get_demands(&mut *pid, *vstate, demands);
    }

    demands
}
