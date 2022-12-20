/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

pub mod pids {

    #[derive(Debug,Clone)]
    pub enum PidController {

        AnglePid { 
            x: f32,
            y: f32,
            s: f32 
        },

        AltHoldPid { 
            k_p : f32,
            k_i: f32, 
            in_band_prev: bool,
            error_integral: f32,
            altitude_target: f32
        },
    }

    pub fn makeAltHold(k_p: f32, k_i: f32) {

        PidController::AltHoldPid {
            k_p : k_p,
            k_i: k_i,
            in_band_prev: false,
            error_integral: 0.0,
            altitude_target: 0.0
        }
    }

    pub fn get_demands(t: &mut PidController, dx: f32, dy: f32) {

        match *t {

            PidController::AnglePid {
                ref mut x,
                ref mut y,
                s: _
            } => {

                *x += dx;
                *y += dy
            },

            PidController::AltHoldPid {
                k_p,
                k_i, 
                ref mut in_band_prev,
                ref mut error_integral,
                ref mut altitude_target
            } => {

                *in_band_prev = false
            },
        }
    }

} // mod pids
