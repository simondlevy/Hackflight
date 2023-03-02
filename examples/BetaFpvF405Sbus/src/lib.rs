/*
   Copyright (c) 2022 Simon D. Levy

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

#![no_std]

use panic_halt as _; 

/*
#[no_mangle]
pub fn quadxbf_mix(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> (f32, f32, f32, f32)
{
    // right rear
    let m1 = throttle - roll + pitch + yaw; 

    // right front
    let m2 = throttle - roll - pitch - yaw; 

    // left rear
    let m3 = throttle + roll + pitch - yaw; 

    // left front
    let m4 = throttle + roll - pitch + yaw;

    (m1, m2, m3, m4)
}
*/

#[no_mangle]
pub fn quadxbf_mix(t: f32) -> f32
{
    t
}
