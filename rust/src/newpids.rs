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

}

