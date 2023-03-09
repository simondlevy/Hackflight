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

use core::f32::consts::PI;
use micromath::F32Ext;

// http://http.developer.nvidia.com/Cg/acos.html 
// Handbook of Mathematical Functions M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
pub fn acos_approx(x : f32) -> f32 {

    let xa = x.abs();

    let result =
        (1. - xa).sqrt() *
        (1.5707288 + xa *
         (-0.2121144 + xa *
          (0.0742610 + (-0.0187293 * xa))));

    if x < 0.  { PI - result } else { result }
}

pub fn atan2_approx(y: f32, x: f32) -> f32 {

    const ATAN_POLY_COEF1 : f32 = 3.14551665884836e-07;
    const ATAN_POLY_COEF2 : f32 = 0.99997356613987;
    const ATAN_POLY_COEF3 : f32 = 0.14744007058297684;
    const ATAN_POLY_COEF4 : f32 = 0.3099814292351353;
    const ATAN_POLY_COEF5 : f32 = 0.05030176425872175;
    const ATAN_POLY_COEF6 : f32 = 0.1471039133652469;
    const ATAN_POLY_COEF7 : f32 = 0.6444640676891548;

    let abs_x = x.abs();

    let abs_y = y.abs();

    let a  = if abs_x > abs_y { abs_x } else { abs_y };

    let b = if a > 0. {
        (if abs_x < abs_y { abs_x } else  { abs_y }) / a 
    } else { 
        0. 
    };

    let c = -((((ATAN_POLY_COEF5 * b - ATAN_POLY_COEF4) * b - ATAN_POLY_COEF3) *
               b - ATAN_POLY_COEF2) * b - ATAN_POLY_COEF1) / 
        ((ATAN_POLY_COEF7 * b + ATAN_POLY_COEF6) * b + 1.);

    let d = if abs_y > abs_x { PI / 2. - c } else { c };

    let e = if x < 0.  { PI - d } else { d };

    if y < 0. { e } else {-e }
}

pub fn inv_sqrt(x: f32) -> f32 {

    1. / x.sqrt()
}

pub fn sq(x: f32) -> f32 {
    x * x
}

pub fn deg2rad(d : f32) -> f32 {

    d * PI / 180.
}

pub fn rad2deg(r : f32) -> f32 {

    r * 180. / PI
}
