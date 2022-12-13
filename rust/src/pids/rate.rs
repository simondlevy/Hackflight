/*
   Roll/pitch rate PID controller

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

use datatypes::datatypes::Demands;
use datatypes::datatypes::VehicleState;

use utils::utils::constrain_abs;

#[derive(Clone)]
struct AxisPid {
    error_integral: f32,
    previous_error: f32
}

#[derive(Clone)]
pub struct RatePid {
    roll: AxisPid,
    pitch: AxisPid
}

fn run_axis(demand: f32, angvel: f32, axis_pid: AxisPid) -> (f32, AxisPid) {

    const KP: f32 = 0.225;
    const KI: f32 = 0.001875;
    const KD: f32 = 0.375;

    const WINDUP_MAX: f32 = 6.0;

    let error = demand - angvel;

    let error_integral = constrain_abs(axis_pid.error_integral, WINDUP_MAX);

    let error_derivative = error - axis_pid.previous_error;

    let new_demand = KP * error + KI * error_integral + KD * error_derivative;

    let new_pid = AxisPid { error_integral:error_integral, previous_error:error };

    (new_demand, new_pid)
}

pub fn run(
    demands:Demands,
    vstate:&VehicleState,
    pid: RatePid) -> (Demands, RatePid) {

    let (roll_demand, roll_pid) = run_axis(demands.roll, vstate.dphi, pid.roll);

    // Pitch demand is nose-down positive, so we negate pitch-forward rate
    // (nose-down negative)
    let (pitch_demand, pitch_pid) = run_axis(demands.pitch, -vstate.dtheta, pid.pitch);

    let new_demands = Demands {
        throttle:demands.throttle,
        roll:roll_demand,
        pitch:pitch_demand,
        yaw:demands.yaw
    };

    let new_pid = RatePid { roll: roll_pid, pitch: pitch_pid };

    (new_demands, new_pid)
}

fn make_axis(error_integral:f32, previous_error:f32) -> AxisPid {
    AxisPid {
        error_integral:error_integral,
        previous_error:previous_error
    }
}

pub fn new() -> RatePid {
    RatePid {roll:make_axis(0.0, 0.0), pitch:make_axis(0.0, 0.0) }
}
