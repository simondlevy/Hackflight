/*
   Hackflight mixuer support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::Demands;
use crate::Motors;

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
pub fn quad_xbf(demands:Demands) -> Motors {

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
