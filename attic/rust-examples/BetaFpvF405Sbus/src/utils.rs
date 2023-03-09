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

pub fn constrain(amt: f32, low: f32, high: f32) -> f32 {

    if amt < low { low } else if amt > high { high } else { amt }
}

pub fn max(a: f32, b: f32) -> f32 {

    if a > b { a } else { b }
}

pub fn min(a: f32, b: f32) -> f32 {

    if a < b { a } else { b }
}
