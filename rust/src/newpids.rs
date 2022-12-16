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

    fn get_demands(pid: &PidController) -> Demands {

        match pid {

            PidController::AnglePid {
                dyn_lpf_previous_quantized_throttle,  
                feedforward_lpf_initialized,
                sum,
            } => { 
                get_angle_demands(
                    dyn_lpf_previous_quantized_throttle,  
                    feedforward_lpf_initialized,
                    sum) 
            },

            PidController::AltitudePid { x, y, r } => {
                get_alt_hold_demands(x, y, r)
            },
        }
    }

    fn get_angle_demands(
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
                    x: &f32,  
                    y: &f32,
                    z: &f32) -> Demands  {
        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }
}

