/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod altitude;
pub mod rate;
pub mod yaw;

pub mod angle;
pub mod althold;

pub mod pids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;

    use crate::pids::altitude as alt_pid;

    use crate::pids::altitude::AltitudePid;

    use crate::pids::yaw as yaw_pid;
    use crate::pids::yaw::YawPid;

    use crate::pids::rate as rate_pid;
    use crate::pids::rate::RatePid;

    use crate::pids::angle::AnglePid;
    use crate::pids::angle::makeAnglePid;

    use crate::pids::althold::AltHoldPid;
    use crate::pids::althold::makeAltHoldPid;

    #[derive(Clone)]
    pub enum PidController {

        Angle { ap : AnglePid, },

        AltHold { ahp : AltHoldPid, },
    }

    pub fn makeAnglePidController( 
        kRateP: f32,
        kRateI: f32,
        kRateD: f32,
        kRateF: f32,
        kLevelP: f32) -> PidController {

        PidController::Angle {ap : makeAnglePid(kRateP, kRateI, kRateD, kRateF, kLevelP)}
    }

    pub fn makeAltHoldPidController(kP: f32, kI: f32) -> PidController {

        PidController::AltHold {ahp : makeAltHoldPid(kP, kI)}
    }


    ///////////////////////////////////////////////////////////////////////////

    #[derive(Copy,Clone)]
    pub struct Controller {

        alt: AltitudePid,
        yaw: YawPid,
        rate: RatePid
    }

    pub fn new_controller() -> Controller {

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

