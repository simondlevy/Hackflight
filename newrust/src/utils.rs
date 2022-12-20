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

pub fn _deg2rad(x: f32) -> f32 {
    3.1415928 * x / 180.0
}
