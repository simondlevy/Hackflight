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

use std::f32::consts::PI;

use crate::clock::DT;

// Pt1 --------------------------------------------------------------------

#[derive(Clone,Copy)]
pub struct Pt1 {

    state: f32,
    k: f32
}

pub fn apply_pt1(mut filter: Pt1, input: f32) -> f32 {

    filter.state = filter.state + filter.k * (input - filter.state);

    filter.state
}

pub fn make_pt1(f_cut: f32) -> Pt1 {

    let k = compute_pt1_gain(f_cut);

    Pt1 {state: 0.0, k: k }
}

pub fn adjust_pt1_gain(mut filter: Pt1, f_cut: f32)
{
    filter.k = compute_pt1_gain(f_cut);
}


fn compute_pt1_gain(f_cut:f32) -> f32 {

    compute_gain(1.0, f_cut)
}

// Pt2 --------------------------------------------------------------------

#[derive(Clone,Copy)]
pub struct Pt2 {

    state: f32,
    state1: f32,
    k: f32
}

pub fn apply_pt2(mut filter: Pt2, input: f32) -> f32 {

    let state1 = filter.state1 + filter.k * (input - filter.state1);

    filter.state = filter.state + filter.k * (state1 - filter.state);

    filter.state
}

pub fn make_pt2(f_cut: f32) -> Pt2 {

    let k = compute_gain_with_order(2.0, f_cut);

    Pt2 {state: 0.0, state1: 0.0, k: k }
}

// Pt3 --------------------------------------------------------------------

#[derive(Clone,Copy)]
pub struct Pt3 {

    state: f32,
    state1: f32,
    state2: f32,
    k: f32
}

pub fn apply_pt3(mut filter: Pt3, input: f32) -> f32 {

    let state1 = filter.state1 + filter.k * (input - filter.state1);
    let state2 = filter.state2 + filter.k * (state1 - filter.state2);

    filter.state = filter.state + filter.k * (state2 - filter.state);

    filter.state
}

pub fn make_pt3(f_cut: f32) -> Pt3 {

    let k = compute_gain_with_order(3.0, f_cut);

    Pt3 {state: 0.0, state1: 0.0, state2: 0.0, k: k }
}


// Helpers --------------------------------------------------------------------

fn compute_gain_with_order(order: f32, f_cut: f32) -> f32 {

    let two: f32 = 2.0;
    let order_cutoff_correction = 1.0 / (two.powf(1.0 / order) - 1.0).sqrt();

    compute_gain(order_cutoff_correction, f_cut)
}

fn compute_gain(order_cutoff_correction: f32, f_cut: f32) -> f32 {

    let rc = 1.0 / (2.0 * order_cutoff_correction * PI * f_cut);

    DT / (rc + DT)
}
