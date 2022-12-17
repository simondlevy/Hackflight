/*
   Roll/pitch rate PID controller

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

use crate::datatypes::Demands;
use crate::datatypes::VehicleState;

use crate::filters;
use crate::utils;

#[derive(Clone,Copy)]
struct Axis {

    previousSetpoint : f32,
    integral : f32
}

#[derive(Clone)]
struct CyclicAxis {

    axis: Axis,
    dtermLpf1 : filters::Pt1,
    dtermLpf2 : filters::Pt1,
    dMinLpf: filters::Pt2,
    dMinRange: filters::Pt2,
    windupLpf: filters::Pt1,
    previousDterm: f32
}

#[derive(Clone)]
pub struct AnglePid {
    kRateP: f32,
    kRateI: f32,
    kRateD: f32,
    kRateF: f32,
    kLevelP: f32,
    roll : CyclicAxis,
    pitch : CyclicAxis,
    dynLpfPreviousQuantizedThrottle: i32,  
    feedforwardLpfInitialized: bool,
    sum: f32,
    ptermYawLpf: filters::Pt1
}

pub fn makeAnglePid( 
    kRateP: f32,
    kRateI: f32,
    kRateD: f32,
    kRateF: f32,
    kLevelP: f32) -> AnglePid {

    const YAW_LOWPASS_HZ: f32 = 100.0;

    AnglePid {
        kRateP: kRateP, 
        kRateI: kRateI, 
        kRateD: kRateD, 
        kRateF: kRateF, 
        kLevelP: kLevelP, 
        roll: makeCyclicAxis(),
        pitch: makeCyclicAxis(),
        dynLpfPreviousQuantizedThrottle: 0,
        feedforwardLpfInitialized: false,
        sum: 0.0,
        ptermYawLpf : filters::makePt1(YAW_LOWPASS_HZ)
    }
}

pub fn getAngleDemands(
    mut pid: &AnglePid,
    demands: &Demands,
    vstate: &VehicleState,
    ) -> Demands  {

    Demands {throttle: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0 }
}

fn makeCyclicAxis() -> CyclicAxis {

    const DTERM_LPF1_DYN_MIN_HZ: f32 = 75.0;
    const DTERM_LPF1_DYN_MAX_HZ: f32 = 150.0;
    const DTERM_LPF2_HZ: f32 = 150.0;
    const D_MIN_LOWPASS_HZ: f32 = 35.0;  
    const ITERM_RELAX_CUTOFF: f32 = 15.0;
    const D_MIN_RANGE_HZ: f32 = 85.0;  

    CyclicAxis {
        axis: makeAxis(),
        dtermLpf1 : filters::makePt1(DTERM_LPF1_DYN_MIN_HZ),
        dtermLpf2 : filters::makePt1(DTERM_LPF2_HZ),
        dMinLpf: filters::makePt2(D_MIN_LOWPASS_HZ),
        dMinRange: filters::makePt2(D_MIN_RANGE_HZ),
        windupLpf: filters::makePt1(ITERM_RELAX_CUTOFF),
        previousDterm: 0.0 }
}

fn makeAxis() -> Axis {

    Axis { previousSetpoint: 0.0, integral: 0.0 }
}

///////////////////////////////////////////////////////////////////////////////////

#[derive(Copy,Clone)]
struct AxisPid {
    error_integral: f32,
    previous_error: f32
}

#[derive(Copy,Clone)]
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

    let error_integral = utils::constrain_abs(axis_pid.error_integral, WINDUP_MAX);

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
