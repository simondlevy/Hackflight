/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/


pub mod newpids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;

    #[derive(Debug,Clone)]
    enum PidController {

        AnglePid {
            k_rate_p: f32,
            k_rate_i: f32,
            k_rate_d: f32,
            k_rate_f: f32,
            k_level_p: f32,
            dyn_lpf_previous_quantized_throttle: i32,  
            feedforward_lpf_initialized: bool,
            sum: f32,
        },

        AltitudePid {
            k_p: f32,
            k_i: f32,
            in_band_prev: bool,
            error_integral: f32,
            altitude_target: f32 
        },
    }

    fn get_demands(pid: &PidController) -> Demands {

        match pid {

            PidController::AnglePid {
                k_rate_p,
                k_rate_i,
                k_rate_d,
                k_rate_f,
                k_level_p,
                dyn_lpf_previous_quantized_throttle,  
                feedforward_lpf_initialized,
                sum,
            } => { 
                get_angle_demands(
                    k_rate_p,
                    k_rate_i,
                    k_rate_d,
                    k_rate_f,
                    k_level_p,
                    dyn_lpf_previous_quantized_throttle,  
                    feedforward_lpf_initialized,
                    sum) 
            },

            PidController::AltitudePid {
                k_p,
                k_i,
                in_band_prev,  
                error_integral,
                altitude_target,
            } => { 
                get_alt_hold_demands(
                    k_p,
                    k_i,
                    in_band_prev,  
                    error_integral,
                    altitude_target) 
            },

        }
    }

    fn get_angle_demands(
                    k_rate_p: &f32,
                    k_rate_i: &f32,
                    k_rate_d: &f32,
                    k_rate_f: &f32,
                    k_level_p: &f32,
                    dyn_lpf_previous_quantized_throttle: &i32,  
                    feedforward_lpf_initialized: &bool,
                    sum: &f32) -> Demands  {
        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }

    fn get_alt_hold_demands(
            k_p: &f32,
            k_i: &f32,
            in_band_prev: &bool,
            error_integral: &f32,
            altitude_target: &f32) -> Demands  {
        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }
}

