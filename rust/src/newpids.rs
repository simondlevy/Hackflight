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
            dyn_lpf_previous_quantized_throttle: i32,  
            feedforward_lpf_initialized: bool,
            sum: f32,
        },

        AltitudePid {
            x: f32,
            y: f32,
            r: f32 
        },
    }

    fn get_demands(pid: &PidController) -> f32 {

        match pid {

            PidController::AnglePid {
                dyn_lpf_previous_quantized_throttle: _,  
                feedforward_lpf_initialized: _,
                sum: _,
            } => { 0. },

            PidController::AltitudePid { x:_, y:_, r } => {
                std::f32::consts::PI*r*r
            },
        }
    }
}

