/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */


#[derive(Debug,Clone)]
pub enum PidController {

    AnglePid { 
        k_rate_p: f32,
        k_rate_i: f32,
        k_rate_d: f32,
        k_rate_f: f32,
        k_level_p: f32,
        //roll : CyclicAxis,
        //pitch : CyclicAxis,
        //yaw: Axis,
        dyn_lpf_previous_quantized_throttle: i32,  
        //pterm_yaw_lpf: filters::Pt1
    },

        AltHoldPid { 
            k_p : f32,
            k_i: f32, 
            in_band_prev: bool,
            error_integral: f32,
            altitude_target: f32
        },
}

pub fn make_angle(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32 ) -> PidController {

    PidController::AnglePid {
        k_rate_p: k_rate_p,
        k_rate_i: k_rate_i,
        k_rate_d: k_rate_d,
        k_rate_f: k_rate_f,
        k_level_p: k_level_p,
        dyn_lpf_previous_quantized_throttle: 0  
    }
}

pub fn make_alt_hold(k_p: f32, k_i: f32) -> PidController {

    PidController::AltHoldPid {
        k_p : k_p,
        k_i: k_i,
        in_band_prev: false,
        error_integral: 0.0,
        altitude_target: 0.0
    }
}

pub fn get_demands(t: &mut PidController, _dx: f32, _dy: f32) {

    match *t {

        PidController::AnglePid {
            k_rate_p: _,
            k_rate_i: _,
            k_rate_d: _,
            k_rate_f: _,
            k_level_p: _,
            ref mut dyn_lpf_previous_quantized_throttle
        } => {

            *dyn_lpf_previous_quantized_throttle = 0
        },

        PidController::AltHoldPid {
            k_p: _,
            k_i: _, 
            ref mut in_band_prev,
            ref mut error_integral,
            ref mut altitude_target
        } => {

            *in_band_prev = false;
            *error_integral = 0.0;
            *altitude_target = 0.0;
        },
    }
}
