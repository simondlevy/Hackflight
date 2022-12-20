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

use crate::Demands;
use crate::Mixer;
use crate::Motors;

pub struct QuadXbf {

}

impl Mixer for QuadXbf {

    fn get_motors(&self, demands: &Demands) -> Motors {
        
        /*
        println!("thr={:.6}  rol={:.6}  pit={:.6}  yaw={:.6}",  
            demands.throttle, demands.roll, demands.pitch, demands.yaw);*/

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
