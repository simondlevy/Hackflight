/*
   Hackflight PID controller support

   Copyright (C) 2022 Simon D. Levy

   MIT License
*/

pub mod newpids {

    use crate::datatypes::Demands;
    use crate::datatypes::VehicleState;
    use crate::utils::constrain_abs;
    use crate::filters;
    use crate::utils::DT;
    use crate::utils::constrain_f;

    // Angle ---------------------------------------------------------------


    fn getAngleDemands(
        mut pid: &AnglePid, demands: &Demands, vstate: &VehicleState) -> Demands  {

        // minimum of 5ms between updates
        const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u16 = 5000; 

        const DYN_LPF_THROTTLE_STEPS: u16 = 100;

        const ITERM_WINDUP_POINT_PERCENT: u8 = 85;        

        const D_MIN: u8 = 30;
        const D_MIN_GAIN: u8 = 37;
        const D_MIN_ADVANCE: u8 = 20;

        const FEEDFORWARD_MAX_RATE_LIMIT: u8 = 90;

        const DYN_LPF_CURVE_EXPO: u8 = 5;

        // PT2 lowpass cutoff to smooth the boost effect
        const D_MIN_GAIN_FACTOR: f32  = 0.00008;
        const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;

        const RATE_ACCEL_LIMIT: f32 = 0.0;
        const YAW_RATE_ACCEL_LIMIT: u16 = 0;

        const OUTPUT_SCALING: f32 = 1000.0;
        const  LIMIT_YAW: u16 = 400;
        const  LIMIT: u16 = 500;

        let roll_demand  = rescale(demands.roll);
        let pitch_demand = rescale(demands.pitch);
        let yaw_demand   = rescale(demands.yaw);

        let maxVelocity = RATE_ACCEL_LIMIT * 100.0 * DT;

        let roll = 
            updateCyclic(pid, roll_demand, vstate.phi, vstate.dphi, &pid.roll, maxVelocity);

        let pitch = 
            updateCyclic(pid, pitch_demand, vstate.theta, vstate.dtheta, &pid.pitch, maxVelocity);

        Demands { 
            throttle : 0.0,
            roll : 0.0,
            pitch : 0.0,
            yaw : 0.0
        }
    }

    // [-1,+1] => [-670,+670] with nonlinearity
    fn rescale(command: f32) -> f32 {

        const CTR: f32 = 0.104;

        let expof = command * command.abs();
        let angleRate = command * CTR + (1.0 - CTR) * expof;

        670.0 * angleRate
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

    fn accelerationLimit(axis: Axis, currentSetpoint: f32, maxVelocity: f32) -> f32 {

        let currentVelocity = currentSetpoint - axis.previousSetpoint;

        if currentVelocity.abs() > maxVelocity 
        { if currentVelocity > 0.0 
            { axis.previousSetpoint + maxVelocity } 
            else { axis.previousSetpoint - maxVelocity } 
        }
        else { currentSetpoint }
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

    fn applyItermRelax(
        mut cyclicAxis: &CyclicAxis,
        iterm: f32,
        currentSetpoint: f32,
        itermErrorRate: f32) -> f32
    {
        // Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
        const ITERM_RELAX_SETPOINT_THRESHOLD: f32 = 40.0;

        // XXX need to use newWindupLpf
        let (setpointLpf, newWindupLpf) =
            filters::applyPt1(cyclicAxis.windupLpf, currentSetpoint);

        let setpointHpf = (currentSetpoint - setpointLpf).abs();

        let itermRelaxFactor =
            (1.0 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD).max(0.0);

        let isDecreasingI =
            ((iterm > 0.0) && (itermErrorRate < 0.0)) ||
            ((iterm < 0.0) && (itermErrorRate > 0.0));

        itermErrorRate * (if !isDecreasingI  {itermRelaxFactor} else {1.0} )
    }


    fn updateCyclic(
        mut pid: &AnglePid,
        demand: f32,
        angle: f32,
        angvel: f32,
        mut cyclicAxis: &CyclicAxis,
        maxVelocity: f32) -> f32
    {
        const ITERM_LIMIT: f32 = 400.0;

        let axis = cyclicAxis.axis.clone();

        let currentSetpoint =
            if { maxVelocity > 0.0 } { accelerationLimit(axis, demand, maxVelocity) } else { demand };

        // XXX axis.previousSetpoint = newSetpoint;

        let newSetpoint = levelPid(pid.kLevelP, currentSetpoint, angle);

        // -----calculate error rate
        let errorRate = newSetpoint - angvel;

        let itermErrorRate = applyItermRelax(cyclicAxis, axis.integral, newSetpoint, errorRate);

        // -----calculate P component
        let P = pid.kRateP * errorRate;

        // -----calculate I component XXX need to store in axis
        let I = constrain_f(axis.integral + (pid.kRateI * DT) * itermErrorRate,
                           -ITERM_LIMIT, ITERM_LIMIT);

        /*
        // -----calculate D component
        let dterm = cyclicAxis.dtermLpf2.apply(cyclicAxis.dtermLpf1.apply(angvel));

        const auto D = m_kRateD > 0 ?  computeDerivative(cyclicAxis, 0, dterm) : 0;

        cyclicAxis.previousDterm = dterm;

        // -----calculate feedforward component
        const auto F =
        m_kRateF > 0 ?
        computeFeedforward(newSetpoint, 670, 0) :
        0;

        return P + axis->I + D + F;
         */

        0.0

    } // updateCyclic

} // mod newpids
