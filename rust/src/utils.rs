/*
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

pub fn constrain_f(val: f32, lo: f32, hi: f32) -> f32 {
    if val < lo {lo} else if val > hi {hi} else {val }
}

pub fn constrain(val: f32, lo: f32, hi: f32) -> f32 {
    if val  < lo {lo} else if val > hi {hi} else {val}
}

pub fn constrain_abs(val : f32, limit : f32) -> f32 {
    constrain(val, -limit, limit)
}

pub fn rescale(val: f32, oldmin: f32, oldmax: f32, newmin: f32, newmax: f32) -> f32 {

    newmin + (val - oldmin) / (oldmax - oldmin) * (newmax - newmin)
}

pub fn rad2deg(rad: f32) -> f32 {

    180.0 * rad / std::f32::consts::PI
}
