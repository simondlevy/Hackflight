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

    #[derive(Clone)]
    pub enum PidController {

        Angle { ap : AnglePid, },

        AltitudeHold { ahp : AltitudeHoldPid, },
    }

    fn getDemands(
        pid: &PidController,
        dUsec: &u32,
        demands: &Demands,
        vstate: &VehicleState,
        reset: &bool) -> Demands {

        match pid {

            PidController::Angle { ap } => { 
                getAngleDemands(ap, demands, vstate)
            },

            PidController::AltitudeHold { ahp } => {
                getAltitudeHoldDemands(ahp, demands, vstate, reset)
            }
        }
    }

    // Angle ---------------------------------------------------------------

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

    pub fn makeAnglePidController( 
        kRateP: f32,
        kRateI: f32,
        kRateD: f32,
        kRateF: f32,
        kLevelP: f32) -> PidController {

        const YAW_LOWPASS_HZ: f32 = 100.0;

        PidController::Angle {
            ap : AnglePid {
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

    fn getAngleDemands(pid: &AnglePid, demands: &Demands, vstate: &VehicleState) -> Demands  {

        // minimum of 5ms between updates
        const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u16 = 5000; 

        const DYN_LPF_THROTTLE_STEPS: u16 = 100;

        // Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
        const ITERM_RELAX_SETPOINT_THRESHOLD: u8 = 40;

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
        const ITERM_LIMIT: u16 = 400;

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

        let newSetpoint = 
            if currentVelocity.abs() > maxVelocity 
            { if currentVelocity > 0.0 
                { axis.previousSetpoint + maxVelocity } 
                else { axis.previousSetpoint - maxVelocity } 
            }
            else { currentSetpoint };

        // XXX axis.previousSetpoint = newSetpoint;

        newSetpoint
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
        cyclicAxis: &CyclicAxis,
        iterm: f32,
        currentSetpoint: f32,
        itermErrorRate: f32) -> f32
    {
        /*
        let setpointLpf = cyclicAxis.windupLpf.apply(currentSetpoint);

        const float setpointHpf = fabsf(currentSetpoint - setpointLpf);

        const auto itermRelaxFactor =
            fmaxf(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);

        const auto isDecreasingI =
            ((iterm > 0) && (itermErrorRate < 0)) ||
            ((iterm < 0) && (itermErrorRate > 0));

        return itermErrorRate * (!isDecreasingI ? itermRelaxFactor : 1);
        */
        0.0
    }


    fn updateCyclic(
        pid: &AnglePid,
        demand: f32,
        angle: f32,
        angvel: f32,
        cyclicAxis: &CyclicAxis,
        maxVelocity: f32) -> f32
    {
        let axis = cyclicAxis.axis.clone();

        let currentSetpoint =
            if { maxVelocity > 0.0 } { accelerationLimit(axis, demand, maxVelocity) } else { demand };

        let newSetpoint = levelPid(pid.kLevelP, currentSetpoint, angle);

        // -----calculate error rate
        let errorRate = newSetpoint - angvel;

        let itermErrorRate = applyItermRelax(cyclicAxis, axis.integral, newSetpoint, errorRate);

        /*
        // -----calculate P component
        const auto P = m_kRateP * errorRate;

        // -----calculate I component
        axis->I =
        constrain_f(axis->I + (m_kRateI * Clock::DT()) * itermErrorRate,
        -ITERM_LIMIT, +ITERM_LIMIT);

        // -----calculate D component
        const auto dterm =
        cyclicAxis.dtermLpf2.apply(cyclicAxis.dtermLpf1.apply(angvel));
        const auto D =
        m_kRateD > 0 ?
        computeDerivative(cyclicAxis, 0, dterm) :
        0;

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


    // AltHoldPid -------------------------------------------------------------

    #[derive(Clone)]
    pub struct AltitudeHoldPid {
        kP : f32,
        kI: f32, 
        inBandPrev: bool,
        errorIntegral: f32,
        altitudeTarget: f32
    }

    pub fn makeAltitudeHoldPidController(kP: f32, kI: f32) -> PidController {

        PidController::AltitudeHold {
            ahp : AltitudeHoldPid {
                kP: kP, 
                kI: kI, 
                inBandPrev: false,
                errorIntegral: 0.0,
                altitudeTarget: 0.0 
            }
        }
    }

    fn getAltitudeHoldDemands(
        pid: &AltitudeHoldPid,
        demands: &Demands,
        vstate: &VehicleState,
        reset: &bool
    ) -> Demands  {

        const ALTITUDE_MIN   :f32 = 1.0;
        const PILOT_VELZ_MAX :f32 = 2.5;
        const STICK_DEADBAND :f32 = 0.2;
        const WINDUP_MAX     :f32 = 0.4;

        let altitude = vstate.z;
        let dz = vstate.dz;

        // [0,1] => [-1,+1]
        let sthrottle = 2.0 * demands.throttle - 1.0; 

        // Is stick demand in deadband, above a minimum altitude?
        let in_band = sthrottle.abs() < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

        // Reset controller when moving into deadband above a minimum altitude
        let got_target = in_band && !pid.inBandPrev;
        let errorIntegral = if got_target || *reset { 0.0 } else { pid.errorIntegral };

        let inBandPrev = in_band;

        let altitudeTarget = if *reset { 0.0 } else { pid.altitudeTarget };

        // Target velocity is a setpoint inside deadband, scaled constant outside
        let target_velocity =
            if in_band {altitudeTarget - altitude } else { PILOT_VELZ_MAX * sthrottle};

        // Compute error as scaled target minus actual
        let error = target_velocity - dz;

        // Compute I term, avoiding windup
        let errorIntegral = constrain_abs(errorIntegral + error, WINDUP_MAX);

        Demands { 
            throttle : demands.throttle + (error * pid.kP + errorIntegral * pid.kI),
            roll : demands.roll,
            pitch : demands.pitch,
            yaw : demands.yaw
            }

    } // getAltitudeHoldDemands

} // mod newpids
