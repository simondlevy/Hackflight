/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod newpids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;

    trait PidControllerTrait {

        fn get_demands_trait(
            &self,
            d_usec: &u32,
            demands: &Demands,
            vstate: &VehicleState,
            reset: &bool) -> Demands; 
        }
} 
