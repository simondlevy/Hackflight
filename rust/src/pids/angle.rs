/*
   Roll/pitch rate PID controller

   Copyright (C) 2022 Simon D. Levy

   MIT License
 */

use crate::datatypes::Demands;
use crate::datatypes::VehicleState;

use crate::filters;
use crate::utils;
use crate::utils::constrain_abs;
use crate::utils::DT;
use crate::utils::constrain_f;

const DTERM_LPF1_DYN_MIN_HZ: f32 = 75.0;
const DTERM_LPF1_DYN_MAX_HZ: f32 = 150.0;
const DTERM_LPF2_HZ: f32 = 150.0;
const D_MIN_LOWPASS_HZ: f32 = 35.0;  
const ITERM_RELAX_CUTOFF: f32 = 15.0;
const D_MIN_RANGE_HZ: f32 = 85.0;  

// minimum of 5ms between updates
const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u16 = 5000; 

const DYN_LPF_THROTTLE_STEPS: u16 = 100;

const ITERM_LIMIT: f32 = 400.0;

const ITERM_WINDUP_POINT_PERCENT: f32 = 85.0;        

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
const ITERM_RELAX_SETPOINT_THRESHOLD: f32 = 40.0;

const D_MIN: u8 = 30;
const D_MIN_GAIN: u8 = 37;
const D_MIN_ADVANCE: u8 = 20;

const FEEDFORWARD_MAX_RATE_LIMIT: f32 = 900.0;

const DYN_LPF_CURVE_EXPO: u8 = 5;

// PT2 lowpass cutoff to smooth the boost effect
const D_MIN_GAIN_FACTOR: f32  = 0.00008;
const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;

const RATE_ACCEL_LIMIT: f32 = 0.0;
const YAW_RATE_ACCEL_LIMIT: f32 = 0.0;

const OUTPUT_SCALING: f32 = 1000.0;
const  LIMIT_YAW: u16 = 400;
const  LIMIT: u16 = 500;


#[derive(Clone)]
pub struct Pid {
    kRateP: f32,
    kRateI: f32,
    kRateD: f32,
    kRateF: f32,
    kLevelP: f32,
    roll : CyclicAxis,
    pitch : CyclicAxis,
    yaw: Axis,
    dynLpfPreviousQuantizedThrottle: i32,  
    feedforwardLpfInitialized: bool,
    sum: f32,
    ptermYawLpf: filters::Pt1
}

pub fn makePid( 
    kRateP: f32,
    kRateI: f32,
    kRateD: f32,
    kRateF: f32,
    kLevelP: f32) -> Pid {

    const YAW_LOWPASS_HZ: f32 = 100.0;

    Pid {
        kRateP: kRateP, 
        kRateI: kRateI, 
        kRateD: kRateD, 
        kRateF: kRateF, 
        kLevelP: kLevelP, 
        roll: makeCyclicAxis(),
        pitch: makeCyclicAxis(),
        yaw: makeAxis(),
        dynLpfPreviousQuantizedThrottle: 0,
        feedforwardLpfInitialized: false,
        sum: 0.0,
        ptermYawLpf : filters::makePt1(YAW_LOWPASS_HZ)
    }
}

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

fn makeCyclicAxis() -> CyclicAxis {

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

pub fn getDemands(
    pid: &mut Pid, demands: &Demands, vstate: &VehicleState, reset: &bool) -> Demands {

    let rollDemand  = rescale(demands.roll);
    let pitchDemand = rescale(demands.pitch);
    let yawDemand   = rescale(demands.yaw);

    let maxVelocity = RATE_ACCEL_LIMIT * 100.0 * DT;

    let roll = 
        updateCyclic(
            &mut pid.roll,
            pid.kLevelP,
            pid.kRateP,
            pid.kRateI,
            pid.kRateD,
            pid.kRateF,
            rollDemand,
            vstate.phi,
            vstate.dphi,
            maxVelocity);

    /*let pitch = 
        updateCyclic(&pid.pitch pitchDemand, vstate.theta, vstate.dtheta, maxVelocity);*/

    let yaw = updateYaw(
        &mut pid.yaw,
        pid.ptermYawLpf,
        pid.kRateP,
        pid.kRateI,
        yawDemand,
        vstate.dpsi);

    Demands { 
        throttle : 0.0,
        roll : 0.0,
        pitch : 0.0,
        yaw : 0.0
    }
}

fn updateYaw(
    axis: &mut Axis,
    ptermLpf: filters::Pt1,
    kp: f32,
    ki: f32,
    demand: f32,
    angvel: f32) -> f32 {

    let maxVelocity = YAW_RATE_ACCEL_LIMIT * 100.0 * DT; 

    // gradually scale back integration when above windup point
    let itermWindupPointInv = 1.0 / (1.0 - (ITERM_WINDUP_POINT_PERCENT / 100.0));

    let dynCi = DT * (if itermWindupPointInv > 1.0
                      {constrain_f(itermWindupPointInv, 0.0, 1.0)}
                      else {1.0});

    let currentSetpoint =
        if maxVelocity > 0.0 {accelerationLimit(axis, demand, maxVelocity)} else {demand};

    let errorRate = currentSetpoint - angvel;

    // -----calculate P component
    let P = filters::applyPt1(ptermLpf, kp * errorRate);

    // -----calculate I component, constraining windup
    axis.integral =
        constrain_f(axis.integral + (ki * dynCi) * errorRate, -ITERM_LIMIT, ITERM_LIMIT);

    P + axis.integral
}

fn accelerationLimit(axis: &mut Axis, currentSetpoint: f32, maxVelocity: f32) -> f32 {

    let currentVelocity = currentSetpoint - axis.previousSetpoint;

    let newSetpoint = 
        if currentVelocity.abs() > maxVelocity 
        { if currentVelocity > 0.0 
            { axis.previousSetpoint + maxVelocity } 
            else { axis.previousSetpoint - maxVelocity } 
        }
        else { currentSetpoint };

    axis.previousSetpoint = newSetpoint;

    newSetpoint
}

// [-1,+1] => [-670,+670] with nonlinearity
fn rescale(command: f32) -> f32 {

    const CTR: f32 = 0.104;

    let expof = command * command.abs();
    let angleRate = command * CTR + (1.0 - CTR) * expof;

    670.0 * angleRate
}

fn levelPid(kLevelP: f32, currentSetpoint: f32, currentAngle: f32) -> f32
{
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection in [-1.0, 1.0]

    const LEVEL_ANGLE_LIMIT: f32 = 45.0;

    let angle = constrain_f(LEVEL_ANGLE_LIMIT * currentSetpoint, -LEVEL_ANGLE_LIMIT, LEVEL_ANGLE_LIMIT);

    let angleError = angle - (currentAngle / 10.0);

    if kLevelP > 0.0  {angleError * kLevelP } else {currentSetpoint}
}

fn updateCyclic(
    cyclicAxis: &mut CyclicAxis,
    kLevelP: f32,
    kRateP: f32,
    kRateI: f32,
    kRateD: f32,
    kRateF: f32,
    demand: f32,
    angle: f32,
    angvel: f32,
    maxVelocity: f32) -> f32
{
    let axis: &mut Axis = &mut cyclicAxis.axis;

    let currentSetpoint =
        if maxVelocity > 0.0
        // {accelerationLimit(&mut cyclicAxis.axis, demand, maxVelocity)}
        {accelerationLimit(axis, demand, maxVelocity)}
        else {demand};

    let newSetpoint = levelPid(kLevelP, currentSetpoint, angle);

    // -----calculate error rate
    let errorRate = newSetpoint - angvel;

    let setpointLpf = filters::applyPt1(cyclicAxis.windupLpf, currentSetpoint);

    let setpointHpf = (currentSetpoint - setpointLpf).abs();

    let itermRelaxFactor =
        (1.0 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD).max(0.0);

    let isDecreasingI =
        ((axis.integral > 0.0) && (errorRate < 0.0)) ||
        ((axis.integral < 0.0) && (errorRate > 0.0));

    // applyItermRelax in original
    let itermErrorRate = errorRate * (if !isDecreasingI  {itermRelaxFactor} else {1.0} );

    // -----calculate P component
    let P = kRateP * errorRate;

    // -----calculate I component XXX need to store in axis
    axis.integral = constrain_f(axis.integral + (kRateI * DT) * itermErrorRate,
                        -ITERM_LIMIT, ITERM_LIMIT);

    // -----calculate D component
    let dterm = filters::applyPt1(
        cyclicAxis.dtermLpf2, 
        filters::applyPt1(cyclicAxis.dtermLpf1, angvel));

    let D = if kRateD > 0.0 {computeDerivative(cyclicAxis, 0.0, dterm)} else {0.0};

    cyclicAxis.previousDterm = dterm;

    // -----calculate feedforward component
    let F = 
        if kRateF > 0.0
        {computeFeedforward(newSetpoint, kRateF, kRateP, 670.0, 0.0)}
        else {0.0};

    0.0 // P + axis.integral + D + F
}

fn computeDerivative(cyclicAxis: &mut CyclicAxis, demandDelta: f32, dterm: f32) -> f32 {
    /*
    // Divide rate change by dT to get differential (ie dr/dt).
    // dT is fixed and calculated from the target PID loop time
    // This is done to avoid DTerm spikes that occur with
    // dynamically calculated deltaT whenever another task causes
    // the PID loop execution to be delayed.
    const float delta = -(dterm - cyclicAxis.previousDterm) * frequency();

    const auto preTpaD = m_k_rate_d * delta;

    const auto dMinPercent = 
        D_MIN > 0 && D_MIN < m_k_rate_d ?
        D_MIN / m_k_rate_d :
        0.0f;

    const auto dMinFactor =
        dMinPercent > 0 ?
        computeDMinFactor(cyclicAxis, dMinPercent, demandDelta, delta) :
        1.0f;

    // Apply the dMinFactor
    return preTpaD * dMinFactor;
    */

    0.0
}

fn computeFeedforward(
    currentSetpoint: f32,
    kRateP : f32,
    kRateF : f32,
    feedforwardMaxRate: f32,
    demandDelta: f32) -> f32 {

    // halve feedforward in Level mode since stick sensitivity is
    // weaker by about half transition now calculated in
    // feedforward.c when new RC data arrives 
    let feedForward = kRateF * demandDelta * frequency();

    let feedforwardMaxRateLimit =
        feedforwardMaxRate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01;

    if feedforwardMaxRateLimit != 0.0 {
        applyFeedforwardLimit(
            feedForward,
            currentSetpoint,
            kRateP,
            feedforwardMaxRateLimit) }
    else {
        feedForward 
    }
}


fn applyFeedforwardLimit(
    value: f32,
    currentSetpoint: f32,
    kRateP : f32,
    maxRateLimit: f32) -> f32 {

    if value * currentSetpoint > 0.0 {
        if currentSetpoint.abs() <= maxRateLimit
        { constrain_f(value, (-maxRateLimit -
                              currentSetpoint) * kRateP,
                              (maxRateLimit - currentSetpoint) * kRateP)}
        else {
            0.0
        }
    }
    else {
        0.0
    }
}

fn frequency() -> f32 {
    1.0 / DT
}
