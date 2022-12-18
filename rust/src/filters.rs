/*
   Filters module

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */


// Pt1 --------------------------------------------------------------------

use crate::utils::constrain_abs;
use crate::utils::DT;

#[derive(Clone,Copy)]
pub struct Pt1 {

    state: f32,
    k: f32
}

pub fn applyPt1(mut filter: Pt1, input: f32) -> f32 {

    filter.state = filter.state + filter.k * (input - filter.state);

    filter.state
}

pub fn makePt1(f_cut: f32) -> Pt1 {

    let k = computeK(1.0, f_cut);

    Pt1 {state: 0.0, k: k }
}

// Pt2 --------------------------------------------------------------------

#[derive(Clone,Copy)]
pub struct Pt2 {

    state: f32,
    state1: f32,
    k: f32
}

pub fn applyPt2(mut filter: Pt2, input: f32) -> f32 {

    let state1 = filter.state1 + filter.k * (input - filter.state1);

    filter.state = filter.state + filter.k * (state1 - filter.state);

    filter.state
}

pub fn makePt2(f_cut: f32) -> Pt2 {

    let cutoff_correction = computeCutoffCorrection(2.0, f_cut);
    let k = computeK(cutoff_correction, f_cut);

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

pub fn applyPt3(mut filter: Pt3, input: f32) -> f32 {

    let state1 = filter.state1 + filter.k * (input - filter.state1);
    let state2 = filter.state2 + filter.k * (state1 - filter.state2);

    filter.state = filter.state + filter.k * (state2 - filter.state);

    filter.state
}

pub fn makePt3(f_cut: f32) -> Pt3 {

    let cutoff_correction = computeCutoffCorrection(3.0, f_cut);
    let k = computeK(cutoff_correction, f_cut);

    Pt3 {state: 0.0, state1: 0.0, state2: 0.0, k: k }
}

// helpers -----------------------------------------------------------------

fn computeK(cutoff_correction:f32, f_cut:f32) -> f32 {

    let rc = 1.0 / (2.0 * cutoff_correction * std::f32::consts::PI * f_cut);

    DT / (rc + DT)
}

fn computeCutoffCorrection(order: f32, f_cut: f32) -> f32 {

    let two: f32 = 2.0;

    1.0 / (two.powf(1.0 / order) - 1.0).sqrt()
}
