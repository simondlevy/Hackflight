/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

use crate::datatypes::Demands;

pub fn quadxbf_mix(demands: Demands) -> (f32, f32, f32, f32) {

    let (thr, rol, pit, yaw) = 
        (demands.throttle, demands.roll, demands.pitch, demands.yaw);

    // right rear
    let m1 = thr - rol + pit - yaw; 

    // right front
    let m2 = thr - rol - pit + yaw; 

    // left rear
    let m3 = thr + rol + pit + yaw; 

    // left front
    let m4 = thr + rol - pit - yaw;

    (m1, m2, m3, m4)
}
