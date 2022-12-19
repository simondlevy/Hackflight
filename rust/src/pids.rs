/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod rate;
pub mod yaw;

pub mod angle;
pub mod althold;

pub mod pids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;

    use crate::pids::althold as alt_pid;

    use crate::pids::althold::AltitudePid;

    use crate::pids::yaw as yaw_pid;
    use crate::pids::yaw::YawPid;

    use crate::pids::rate as rate_pid;
    use crate::pids::rate::RatePid;

    use crate::pids::angle;
    use crate::pids::althold;

    #[derive(Clone)]
    pub enum PidController {

        Angle { ap : angle::Pid, },

        AltHold { ahp : althold::Pid, },
    }

    pub fn get_demands(
        pid: &mut PidController,
        d_usec: &u32,
        demands: &Demands,
        vstate: &VehicleState,
        reset: &bool) -> Demands {

            match pid {

                PidController::Angle { ap } => { 
                    angle::get_demands(ap, d_usec, demands, vstate, reset)
                },

                PidController::AltHold { ahp } => {
                    althold::get_demands(ahp, demands, vstate, reset)
                }
            }
    }

    pub fn make_angle_pid_controller( 
        k_rate_p: f32,
        k_rate_i: f32,
        k_rate_d: f32,
        k_rate_f: f32,
        k_level_p: f32) -> PidController {

            PidController::Angle {ap: angle::make_pid(k_rate_p, k_rate_i, k_rate_d, k_rate_f, k_level_p)}
    }

    pub fn make_alt_hold_pid_controller(k_p: f32, k_i: f32) -> PidController {

        PidController::AltHold {ahp : althold::make_pid(k_p, k_i)}
    }


    ///////////////////////////////////////////////////////////////////////////

    #[derive(Copy,Clone)]
    pub struct Controller {

        alt: AltitudePid,
        yaw: YawPid,
        rate: RatePid
    }

    pub fn make_controller() -> Controller {

        Controller { alt:alt_pid::new(), yaw:yaw_pid::new(), rate:rate_pid::new() }
    }

    pub fn run(
        controller:Controller,
        demands: Demands,
        vehicle_state: VehicleState) -> (Demands, Controller) {

            let (new_demands, new_alt_pid) =
                alt_pid::run(demands, &vehicle_state, controller.alt);

            let (new_demands, new_yaw_pid) =
                yaw_pid::run(new_demands, &vehicle_state, controller.yaw);

            let (new_demands, new_rate_pid) =
                rate_pid::run(new_demands, &vehicle_state, controller.rate);

            let new_controller = Controller {alt:new_alt_pid, yaw:new_yaw_pid, rate:new_rate_pid};

            (new_demands, new_controller)
    }
}

