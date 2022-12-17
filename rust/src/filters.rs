/*
   Filters module

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/



pub mod filters {

    pub struct Pt1 {

        state: f32,
        dt: f32,
        k: f32
    }

    pub fn applyPt1(filter: Pt1, f_cut: f32, input: f32) -> (f32, Pt1) {

        let rc = 1.0 / (2.0 * std::f32::consts::PI * f_cut);
        let k = filter.dt / (rc + filter.dt);
        let state = filter.state + k * (input - filter.state);

        (state, Pt1 {state: state, dt: filter.dt, k: k})
    }

    pub struct Pt2 {

        state: f32,
        dt: f32,
        k: f32
    }

    pub fn applyPt2(filter: Pt2, f_cut: f32, input: f32) -> (f32, Pt2) {

        let rc = 1.0 / (2.0 * std::f32::consts::PI * f_cut);
        let k = filter.dt / (rc + filter.dt);
        let state = filter.state + k * (input - filter.state);

        (state, Pt2 {state: state, dt: filter.dt, k: k})
    }
}
