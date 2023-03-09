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

// Pt1Filter ==================================================================

#[derive(Clone,Copy)]
pub struct Pt1Filter {

    pub state: f32,
    pub dt: f32,
    pub cutoff : f32,
    pub k: f32,
}

impl Pt1Filter {

    pub fn init(&mut self) {

        let rc = 1. / (2. * PI * self.cutoff);
        self.k = self.dt / (rc + self.dt);
    }

    pub fn apply(&mut self, input: f32) -> f32 {

        self.state = self.state + self.k * (input - self.state);
        self.state
    }

    pub fn compute_gain(&mut self, f_cut: f32) {
        // let rc = 1. / (2. * 3.141592 * f_cut);
        let rc = 1. / (2. * PI * f_cut);
        self.k = self.dt / (rc + self.dt);
    }
}

// Pt2Filter ==================================================================

#[derive(Clone,Copy)]
pub struct Pt2Filter {

    pub state: f32,
    pub state1: f32,
    pub cutoff: f32,
    pub dt: f32,
    pub k: f32,
}

impl Pt2Filter {

    pub fn init(&mut self) {
        let order = 2.;
        let order_cutoff_correction = 1. / (2.0.powf(1. / order) - 1.).sqrt();
        let rc = 1. / (2. * order_cutoff_correction * PI * self.cutoff);
        self.k = self.dt / (rc + self.dt);
    }

    pub fn apply(&mut self, input: f32) -> f32 {

        self.state1 = self.state1 + self.k * (input - self.state1);
        self.state = self.state + self.k * (self.state1 - self.state);
        self.state
    }

    pub fn compute_gain(&mut self, f_cut: f32) {
        let order : f32 = 2.0;
        let order_cutoff_correction = 1. / (2.0.powf(1. / order) - 1.0).sqrt();
        let rc = 1. / (2. * order_cutoff_correction * PI * f_cut);
        self.k = self.dt / (rc + self.dt);
    }
}
