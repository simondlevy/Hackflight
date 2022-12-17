/*
   Filters module

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/


pub mod filters {

    use std::f32::consts::PI;

    #[derive(Clone,Copy)]
    pub struct Pt1 {

        state: f32,
        dt: f32,
        k: f32
    }

    pub fn applyPt1(filter: Pt1, f_cut: f32, input: f32) -> (f32, Pt1) {

        let k = computeGainPt1(filter, f_cut);

        let state = filter.state + k * (input - filter.state);

        (state, Pt1 {state: state, dt: filter.dt, k: k})
    }

    fn computeGainPt1(filter: Pt1, f_cut: f32) -> f32 {

        let rc = 1.0 / (2.0 * PI * f_cut);

        filter.dt / (rc + filter.dt)
    }

    /*
    pub struct Pt2 {

        state: f32,
        state1: f32,
        dt: f32,
        k: f32
    }

    pub fn applyPt2(filter: Pt2, f_cut: f32, input: f32) -> (f32, Pt2) {

        let order: f32 = 2.0;
        let order_cutoff_correction = 1 / sqrtf(powf(2, 1.0f / order) - 1);
        let rc = 1.0 / (2.0 * order_cutoff_correction * std::f32::consts::PI * f_cut);
        let k = filter.dt / (rc + filter.dt);

        let state1 = filter.state1 + k * (input - let state1);
        let state = let state + let k * (let state1 - let state);

        (state, Pt2 {state: state, dt: filter.dt, k: k})
    }*/
}
