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
            dynLpfPreviousQuantizedThrottle: i32,  
            feedforwardLpfInitialized: bool,
            sum: f32,
        },

        AltitudePid {
            x: f32,
            y: f32,
            r: f32 
        },
    }

    fn getDemands(pid: &PidController) -> f32 {

        match pid {

            PidController::AnglePid {
                dynLpfPreviousQuantizedThrottle: _,  
                feedforwardLpfInitialized: _,
                sum: _,
            } => { 0. },

            PidController::AltitudePid { x:_, y:_, r } => {
                std::f32::consts::PI*r*r
            },
        }
    }
}

