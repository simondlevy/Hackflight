/*
   Hackflight core algorithm

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod hackflight {

    use datatypes::datatypes::Demands;
    use datatypes::datatypes::Motors;
    use datatypes::datatypes::VehicleState;

    use pids::pids::Controller;
    use pids::pids::run_pids;

    pub fn run_hackflight(
        demands: Demands,
        vehicle_state: VehicleState, 
        pid_controller: Controller,
        mixfun: &dyn Fn(Demands) -> Motors) -> (Motors, Controller) {

        let (demands, new_pid_controller) =
           run_pids(pid_controller, demands, vehicle_state);

        let new_motors = mixfun(demands.clone());
        
        (new_motors, new_pid_controller)

    } // run_hackflight

} // mod hackflight
