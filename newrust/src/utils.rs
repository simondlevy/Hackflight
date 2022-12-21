/*
   Utility functions

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

pub const DT: f32 = 100.0;

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
