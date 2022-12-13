/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod altitude;
pub mod rate;
pub mod yaw;

pub mod pids {

    use datatypes::datatypes::Demands;
    use datatypes::datatypes::VehicleState;

    use pids::altitude as alt_pid;
    use pids::altitude::AltitudePid;

    use pids::yaw as yaw_pid;
    use pids::yaw::YawPid;

    use pids::rate as rate_pid;
    use pids::rate::RatePid;

    pub struct Controller {

        alt: AltitudePid,
        yaw: YawPid,
        rate: RatePid
    }

    pub fn new_controller() -> Controller {

        Controller { alt:alt_pid::new(), yaw:yaw_pid::new(), rate:rate_pid::new() }
    }

    pub fn run_pids(
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

