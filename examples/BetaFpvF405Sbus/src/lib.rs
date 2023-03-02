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

#[no_mangle]
pub extern "C" fn quadxbf_mix(
    throttle: f32, roll: f32, pitch: f32, yaw: f32,
    m1: *mut f32, m2: *mut f32, m3: *mut f32, m4: *mut f32) {

    unsafe {

        // right rear
        *m1 = throttle - roll + pitch - yaw; 

        // right front
        *m2 = throttle - roll - pitch + yaw; 

        // left rear
        *m3 = throttle + roll + pitch + yaw; 

        // left front
        *m4 = throttle + roll - pitch - yaw;
    }
}

