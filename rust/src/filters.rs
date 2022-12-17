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

    pub fn applyPt1(input: f32) -> (f32, Pt1) {

        (0.0, Pt1 {state: 0.0, dt: 0.0, k: 0.0})
    }

    pub fn foo() -> f32 {
        0.0
    }
}
