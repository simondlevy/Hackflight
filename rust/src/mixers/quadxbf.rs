/*
    Mixer values for quad-X frames using Betaflight motor layout:

    4cw   2ccw
    \ /
     ^
    / \
   3ccw  1cw

   Copyright (c) 2022 Simon D. Levy

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
use crate::Mixer;
use crate::Motors;

pub struct QuadXbf {

}

impl Mixer for QuadXbf {

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
