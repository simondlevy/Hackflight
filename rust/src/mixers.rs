/*
   Hackflight mixuer support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod mixers {

    use crate::datatypes::Demands;
    use crate::datatypes::Motors;

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
    pub fn run_quad_xbf(demands:Demands) -> Motors {

        println!("thr={}  rol={}  pit={}  yaw={}",  
            demands.throttle, demands.roll, demands.pitch, demands.yaw);

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
