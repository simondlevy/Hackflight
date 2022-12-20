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

        Motors {m1: 0.0, m2: 0.0, m3:0.0, m4:0.0}

        /*
        Motors {

            // right rear
            m1: demands.throttle - demands.roll + demands.pitch + demands.yaw, 

            // right front
            m2: demands.throttle - demands.roll - demands.pitch - demands.yaw, 

            // left rear
            m3: demands.throttle + demands.roll + demands.pitch - demands.yaw, 

            // left front
            m4: demands.throttle + demands.roll - demands.pitch + demands.yaw  
        }*/
    }
}
