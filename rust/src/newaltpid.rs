/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod newaltpid {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;
    use crate::datatypes::PidControllerTrait;

    const _ALTITUDE_MIN: f32   = 1.0;
    const _PILOT_VELZ_MAX: f32 = 2.5;
    const _STICK_DEADBAND: f32 = 0.2;
    const _WINDUP_MAX: f32     = 0.4;

    pub struct Pid {
        k_p : f32,
        k_i: f32, 
        in_band_prev: bool,
        error_integral: f32,
        altitude_target: f32
    }

    pub fn make(k_p : f32, k_i: f32) -> Pid {

        Pid {
            k_p: k_p,
            k_i: k_i,
            in_band_prev: false,
            error_integral: 0.0,
            altitude_target: 0.0
        }
    }

    impl PidControllerTrait for Pid {

        fn get_demands_trait(
            &self,
            _d_usec: &u32,
            _demands: &Demands,
            _vstate: &VehicleState,
            _reset: &bool) -> Demands {

                Demands { throttle: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0 }
        }
 
    }

} 
