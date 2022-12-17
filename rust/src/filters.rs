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

        (0.0, Pt1 {state: 0.0, dt: 0.0, k: 0.0})
    }
}
