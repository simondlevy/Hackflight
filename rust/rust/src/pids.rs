/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod yaw;
pub mod altitude;

pub mod pids {

    use datatypes::datatypes::Demands;
    use datatypes::datatypes::VehicleState;

    use pids::altitude as altitude_pid;
    use pids::yaw as yaw_pid;

    pub struct Controller {

        x: f32
    }

    pub fn new_controller() -> Controller {

        Controller {x: 0.0}
    }

    pub fn run_pids(
        demands: Demands,
        _vehicle_state: VehicleState) -> Demands {

        demands
    }
}

