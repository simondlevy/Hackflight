/*
   Hackflight core algorithm

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod hackflight {

    use datatypes::datatypes::Demands;
    use datatypes::datatypes::Motors;
    use datatypes::datatypes::VehicleState;

    use pids::altitude::AltitudePid;
    use pids::yaw::YawPid;

    use pids::altitude as altitude_pid;
    use pids::yaw as yaw_pid;

    pub fn run_hackflight(
        demands: Demands,
        vehicle_state: VehicleState, 
        altitude_pid: AltitudePid,
        yaw_pid: YawPid,
        mixfun: &dyn Fn(Demands) -> Motors) -> (Motors, AltitudePid, YawPid) {

        let (demands, new_altitude_pid) =
            altitude_pid::run(demands, &vehicle_state, altitude_pid);

        let (demands, new_yaw_pid) =
            yaw_pid::run(demands, &vehicle_state, yaw_pid);

        println!("yaw demand: {}", demands.yaw);

        let new_motors = mixfun(demands.clone());
        
        (new_motors, new_altitude_pid, new_yaw_pid)

    } // run_hackflight

} // mod hackflight
