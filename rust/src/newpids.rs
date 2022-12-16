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
            x: f32,
            y: f32,
            s: f32 
        },

        AltitudePid {
            x: f32,
            y: f32,
            r: f32 
        },
    }

    fn area(shape: &PidController) -> f32 {
        match shape {
            PidController::AnglePid { x:_, y:_, s } => {
                s*s
            },
            PidController::AltitudePid { x:_, y:_, r } => {
                std::f32::consts::PI*r*r
            },
        }
    }
}

