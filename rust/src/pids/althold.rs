/*
   Altitude-hold PID controller

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::datatypes::Demands;
use crate::datatypes::VehicleState;
use crate::utils;

const ALTITUDE_MIN: f32   = 1.0;
const PILOT_VELZ_MAX: f32 = 2.5;
const STICK_DEADBAND: f32 = 0.2;
const WINDUP_MAX: f32     = 0.4;

#[derive(Clone)]
pub struct Pid {
    kP : f32,
    kI: f32, 
    inBandPrev: bool,
    errorIntegral: f32,
    altitudeTarget: f32
}

pub fn makePid(kP: f32, kI: f32) -> Pid {

    Pid {
        kP: kP, 
        kI: kI, 
        inBandPrev: false,
        errorIntegral: 0.0,
        altitudeTarget: 0.0 
    }
}

pub fn getDemands(
    pid: &mut Pid, demands: &Demands, vstate: &VehicleState, reset: &bool) -> Demands  {

    let altitude = vstate.z;
    let dz = vstate.dz;

    // [0,1] => [-1,+1]
    let sthrottle = 2.0 * demands.throttle - 1.0; 

    // Is stick demand in deadband, above a minimum altitude?
    let inBand = sthrottle.abs() < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

    // Reset controller when moving into deadband above a minimum altitude
    let gotNewTarget = inBand && !pid.inBandPrev;
    let errorIntegral = if gotNewTarget || *reset { 0.0 } else { pid.errorIntegral };

    pid.inBandPrev = inBand;

    pid.altitudeTarget = if *reset { 0.0 } else { pid.altitudeTarget };

    // Target velocity is a setpoint inside deadband, scaled constant outside
    let targetVelocity =
        if inBand {pid.altitudeTarget - altitude } else { PILOT_VELZ_MAX * sthrottle};

    // Compute error as scaled target minus actual
    let error = targetVelocity - dz;

    // Compute I term, avoiding windup
    pid.errorIntegral = utils::constrain_abs(pid.errorIntegral + error, WINDUP_MAX);

    Demands { 
        throttle : demands.throttle + (error * pid.kP + errorIntegral * pid.kI),
        roll : demands.roll,
        pitch : demands.pitch,
        yaw : demands.yaw
    }

} // getDemands


////////////////////////////////////////////////////////////////////////////////////////

#[derive(Copy,Clone)]
pub struct AltitudePid {
    pub error_integral:f32,
    pub inBand:bool,
    pub target:f32
}

pub fn run(
    demands:Demands,
    vstate:&VehicleState,
    pid: AltitudePid) -> (Demands, AltitudePid) {

    const KP: f32 = 0.75;
    const KI: f32 = 1.5;

    let throttle = demands.throttle;

    let altitude = -vstate.z;
    let climb_rate = -vstate.dz;

    // Rescale throttle [0,1] => [-1,+1]
    let sthrottle = throttle; // 2.0 * throttle - 1.0; 

    // Is stick demand in deadband, above a minimum altitude?
    let inBand = sthrottle.abs() < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

    // Zero throttle will reset error integral
    let at_zero_throttle = throttle == 0.0;

    // Reset altitude target at zero throttle
    let altitude_target = if at_zero_throttle {0.0} else {pid.target};

    // If stick just moved into deadband, set new target altitude; otherwise,
    // keep previous
    let new_target = if inBand && !pid.inBand {altitude} else {altitude_target};

    // Target velocity is a setpoint inside deadband, scaled
    // constant outside
    let targetVelocity =
        if inBand {new_target - altitude} else {PILOT_VELZ_MAX * sthrottle};

    // Compute error as scaled target minus actual
    let error = targetVelocity - climb_rate;

    // Compute I term, avoiding windup
    let new_error_integral = utils::constrain_abs(pid.error_integral + error, WINDUP_MAX);

    // Run PI controller
    let correction = error * KP + new_error_integral * KI;

    // Add correction to throttle, constraining output to [0,1]
    let new_demands = Demands {
        throttle:utils::constrain(throttle+correction, 0.0, 1.0),
        roll:demands.roll,
        pitch:demands.pitch,
        yaw:demands.yaw
    };

    // Capture new state of PID controller
    let new_altitude_pid = make(new_error_integral, inBand, new_target);

    (new_demands, new_altitude_pid)
}

fn make(error_integral:f32, inBand:bool, target:f32) -> AltitudePid {
    AltitudePid {
        error_integral: error_integral,
        inBand: inBand,
        target: target
    }
}

pub fn new() -> AltitudePid {
    make(0.0, false, 0.0)
}
