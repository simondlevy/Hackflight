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

use datatypes::datatypes::Demands;
use datatypes::datatypes::Motors;

pub fn run(demands:Demands) -> Motors {

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
