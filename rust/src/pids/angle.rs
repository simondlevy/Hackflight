/*
   Hackflight angle PID controller support

   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

use crate::Demands;
use crate::VehicleState;

use crate::filters;

use crate::utils::constrain_f;

use crate::clock::DT;

const DTERM_LPF1_DYN_MIN_HZ: f32 = 75.0;
const DTERM_LPF1_DYN_MAX_HZ: f32 = 150.0;
const DTERM_LPF2_HZ: f32 = 150.0;
const D_MIN_LOWPASS_HZ: f32 = 35.0;  
const ITERM_RELAX_CUTOFF: f32 = 15.0;
const D_MIN_RANGE_HZ: f32 = 85.0;  

// minimum of 5ms between updates
const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u32 = 5000; 

const DYN_LPF_THROTTLE_STEPS: f32 = 100.0;

const ITERM_LIMIT: f32 = 400.0;

const ITERM_WINDUP_POINT_PERCENT: f32 = 85.0;        

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
const ITERM_RELAX_SETPOINT_THRESHOLD: f32 = 40.0;

const D_MIN: f32 = 30.0;
const D_MIN_GAIN: f32 = 37.0;
const D_MIN_ADVANCE: f32 = 20.0;

const FEEDFORWARD_MAX_RATE_LIMIT: f32 = 900.0;

const DYN_LPF_CURVE_EXPO: f32 = 5.0;

// PT2 lowpass cutoff to smooth the boost effect
const D_MIN_GAIN_FACTOR: f32  = 0.00008;
const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;

const RATE_ACCEL_LIMIT: f32 = 0.0;
const YAW_RATE_ACCEL_LIMIT: f32 = 0.0;

const OUTPUT_SCALING: f32 = 1000.0;
const  LIMIT_CYCLIC: f32 = 500.0; 
const  LIMIT_YAW: f32 = 400.0;

const YAW_LOWPASS_HZ: f32 = 100.0;

#[derive(Clone)]
pub struct Pid { 
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32,
    usec_prev: u32,
    roll : CyclicAxis,
    pitch : CyclicAxis,
    yaw: Axis,
    dyn_lpf_previous_quantized_throttle: i32,  
    pterm_yaw_lpf: filters::Pt1
}

pub fn make(
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    k_level_p: f32) -> Pid {

        Pid {
            k_rate_p: k_rate_p,
            k_rate_i: k_rate_i,
            k_rate_d: k_rate_d,
            k_rate_f: k_rate_f,
            k_level_p: k_level_p,
            usec_prev: 0,
            roll : make_cyclic_axis(),
            pitch : make_cyclic_axis(),
            yaw: make_axis(),
            dyn_lpf_previous_quantized_throttle: 0, 
            pterm_yaw_lpf : filters::make_pt1(YAW_LOWPASS_HZ)
        }
} 

pub fn get_demands(
    pid: &mut Pid,
    usec: &u32,
    demands: &Demands,
    vstate: &VehicleState,
    reset: &bool) -> Demands {

        let d_usec = *usec - pid.usec_prev;
        pid.usec_prev = *usec;

        let roll_demand  = rescale_axis(demands.roll);
        let pitch_demand = rescale_axis(demands.pitch);
        let yaw_demand   = rescale_axis(demands.yaw);

        let max_velocity = RATE_ACCEL_LIMIT * 100.0 * DT;

        let roll = 
            update_cyclic(
                &mut pid.roll,
                pid.k_level_p,
                pid.k_rate_p,
                pid.k_rate_i,
                pid.k_rate_d,
                pid.k_rate_f,
                roll_demand,
                vstate.phi,
                vstate.dphi,
                max_velocity);


        let pitch = 
            update_cyclic(
                &mut pid.pitch,
                pid.k_level_p,
                pid.k_rate_p,
                pid.k_rate_i,
                pid.k_rate_d,
                pid.k_rate_f,
                pitch_demand,
                vstate.theta,
                vstate.dtheta,
                max_velocity);

        let yaw = update_yaw(
            &mut pid.yaw,
            pid.pterm_yaw_lpf,
            pid.k_rate_p,
            pid.k_rate_i,
            yaw_demand,
            vstate.dpsi);

        pid.roll.axis.integral = if *reset { 0.0 } else { pid.roll.axis.integral };
        pid.pitch.axis.integral = if *reset { 0.0 } else { pid.pitch.axis.integral };
        pid.yaw.integral = if *reset { 0.0 } else { pid.yaw.integral };

        if d_usec >= DYN_LPF_THROTTLE_UPDATE_DELAY_US {

            // Quantize the throttle to reduce the number of filter updates
            let quantized_throttle = (demands.throttle * DYN_LPF_THROTTLE_STEPS) as i32; 

            if quantized_throttle != pid.dyn_lpf_previous_quantized_throttle {

                // Scale the quantized value back to the throttle range so the
                // filter cutoff steps are repeatable
                let dyn_lpf_throttle = (quantized_throttle as f32) / DYN_LPF_THROTTLE_STEPS;

                let cutoff_freq = dyn_lpf_cutoff_freq(dyn_lpf_throttle,
                    DTERM_LPF1_DYN_MIN_HZ,
                    DTERM_LPF1_DYN_MAX_HZ,
                    DYN_LPF_CURVE_EXPO);

                init_lpf1(&mut pid.roll, cutoff_freq);
                init_lpf1(&mut pid.pitch, cutoff_freq);

                pid.dyn_lpf_previous_quantized_throttle = quantized_throttle;
            }
        }

        Demands { 
            throttle : demands.throttle,
            roll : constrain_output(roll, LIMIT_CYCLIC),
            pitch : constrain_output(pitch, LIMIT_CYCLIC),
            yaw : constrain_output(yaw, LIMIT_YAW)
        }
    }

#[derive(Clone,Copy)]
struct Axis {

    previous_setpoint : f32,
    integral : f32
}

#[derive(Clone)]
struct CyclicAxis {

    axis: Axis,
    dterm_lpf1 : filters::Pt1,
    dterm_lpf2 : filters::Pt1,
    d_min_lpf: filters::Pt2,
    d_min_range: filters::Pt2,
    windup_lpf: filters::Pt1,
    previous_dterm: f32
}

fn update_cyclic(
    cyclic_axis: &mut CyclicAxis,
    k_level_p: f32,
    k_rate_p: f32,
    k_rate_i: f32,
    k_rate_d: f32,
    k_rate_f: f32,
    demand: f32,
    angle: f32,
    angvel: f32,
    max_velocity: f32) -> f32
{
    let axis: &mut Axis = &mut cyclic_axis.axis;

    let current_setpoint =
        if max_velocity > 0.0
            // {acceleration_limit(&mut cyclic_axis.axis, demand, max_velocity)}
        {acceleration_limit(axis, demand, max_velocity)}
        else {demand};

    let new_setpoint = level_pid(k_level_p, current_setpoint, angle);

    // -----calculate error rate
    let error_rate = new_setpoint - angvel;

    let setpoint_lpf = filters::apply_pt1(cyclic_axis.windup_lpf, current_setpoint);

    let setpoint_hpf = (current_setpoint - setpoint_lpf).abs();

    let iterm_relax_factor =
        (1.0 - setpoint_hpf / ITERM_RELAX_SETPOINT_THRESHOLD).max(0.0);

    let is_decreasing_i =
        ((axis.integral > 0.0) && (error_rate < 0.0)) ||
        ((axis.integral < 0.0) && (error_rate > 0.0));

    // Was applyItermRelax in original
    let iterm_error_rate = error_rate * (if !is_decreasing_i  {iterm_relax_factor} else {1.0} );

    let frequency = 1.0 / DT;

    // Calculate P component --------------------------------------------------
    let pterm = k_rate_p * error_rate;

    // Calculate I component --------------------------------------------------
    axis.integral = constrain_f(axis.integral + (k_rate_i * DT) * iterm_error_rate,
    -ITERM_LIMIT, ITERM_LIMIT);

    // Calculate D component --------------------------------------------------

    let dterm = filters::apply_pt1(
        cyclic_axis.dterm_lpf2, 
        filters::apply_pt1(cyclic_axis.dterm_lpf1, angvel));

    // Divide rate change by dT to get differential (ie dr/dt).
    // dT is fixed and calculated from the target PID loop time
    // This is done to avoid DTerm spikes that occur with
    // dynamically calculated deltaT whenever another task causes
    // the PID loop execution to be delayed.
    let delta = -(dterm - cyclic_axis.previous_dterm) * frequency;

    let pre_t_pa_d = k_rate_d * delta;

    let d_min_percent = if D_MIN > 0.0 && D_MIN < k_rate_d { D_MIN / k_rate_d } else { 0.0 };

    let demand_delta: f32 = 0.0;

    let d_min_gyro_gain = D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;

    let d_min_gyro_factor = (filters::apply_pt2(cyclic_axis.d_min_range, delta)).abs() * d_min_gyro_gain;

    let d_min_setpoint_gain =
        D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR * D_MIN_ADVANCE * frequency /
        (100.0 * D_MIN_LOWPASS_HZ);

    let d_min_setpoint_factor = (demand_delta).abs() * d_min_setpoint_gain;

    let d_min_factor = 
        d_min_percent + (1.0 - d_min_percent) * d_min_gyro_factor.max(d_min_setpoint_factor);

    let d_min_factor_filtered = filters::apply_pt2(cyclic_axis.d_min_lpf, d_min_factor);

    let d_min_factor = d_min_factor_filtered.min(1.0);

    cyclic_axis.previous_dterm = dterm;

    // Apply the d_min_factor
    let dterm = pre_t_pa_d * d_min_factor;

    // Calculate feedforward component -----------------------------------------

    // halve feedforward in Level mode since stick sensitivity is
    // weaker by about half transition now calculated in
    // feedforward.c when new RC data arrives 
    let feed_forward = k_rate_f * demand_delta * frequency;

    let feedforward_max_rate: f32 = 670.0;

    let feedforward_max_rate_limit =
        feedforward_max_rate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01;

    let fterm = if feedforward_max_rate_limit != 0.0 {
        apply_feeedforward_limit(
            feed_forward,
            current_setpoint,
            k_rate_p,
            feedforward_max_rate_limit) }
    else {
        feed_forward 
    };

    pterm + axis.integral + dterm + fterm

} // update_cyclic


fn update_yaw(
    axis: &mut Axis,
    pterm_lpf: filters::Pt1,
    kp: f32,
    ki: f32,
    demand: f32,
    angvel: f32) -> f32 {

        // gradually scale back integration when above windup point
        let iterm_windup_point_inv = 1.0 / (1.0 - (ITERM_WINDUP_POINT_PERCENT / 100.0));

        let dyn_ci = DT * (if iterm_windup_point_inv > 1.0
            {constrain_f(iterm_windup_point_inv, 0.0, 1.0)}
            else {1.0});

        let max_velocity = YAW_RATE_ACCEL_LIMIT * 100.0 * DT; 

        let current_setpoint =
            if max_velocity > 0.0 {acceleration_limit(axis, demand, max_velocity)} else {demand};

        let error_rate = current_setpoint - angvel;

        // -----calculate P component
        let pterm = filters::apply_pt1(pterm_lpf, kp * error_rate);

        // -----calculate I component, constraining windup
        axis.integral =
            constrain_f(axis.integral + (ki * dyn_ci) * error_rate, -ITERM_LIMIT, ITERM_LIMIT);

        pterm + axis.integral
    }


fn make_cyclic_axis() -> CyclicAxis {

    CyclicAxis {
        axis: make_axis(),
        dterm_lpf1 : filters::make_pt1(DTERM_LPF1_DYN_MIN_HZ),
        dterm_lpf2 : filters::make_pt1(DTERM_LPF2_HZ),
        d_min_lpf: filters::make_pt2(D_MIN_LOWPASS_HZ),
        d_min_range: filters::make_pt2(D_MIN_RANGE_HZ),
        windup_lpf: filters::make_pt1(ITERM_RELAX_CUTOFF),
        previous_dterm: 0.0 }
}

fn make_axis() -> Axis {

    Axis { previous_setpoint: 0.0, integral: 0.0 }
}

// [-1,+1] => [-670,+670] with nonlinearity
fn rescale_axis(command: f32) -> f32 {

    const CTR: f32 = 0.104;

    let expof = command * command.abs();
    let angle_rate = command * CTR + (1.0 - CTR) * expof;

    670.0 * angle_rate
}

fn level_pid(k_level_p: f32, current_setpoint: f32, current_angle: f32) -> f32
{
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection in [-1.0, 1.0]

    const LEVEL_ANGLE_LIMIT: f32 = 45.0;

    let angle = constrain_f(LEVEL_ANGLE_LIMIT * current_setpoint,
        -LEVEL_ANGLE_LIMIT, LEVEL_ANGLE_LIMIT);

    let angle_error = angle - (current_angle / 10.0);

    if k_level_p > 0.0  {angle_error * k_level_p } else {current_setpoint}
}

fn acceleration_limit(axis: &mut Axis, current_setpoint: f32, max_velocity: f32) -> f32 {

    let current_velocity = current_setpoint - axis.previous_setpoint;

    let new_setpoint = 
        if current_velocity.abs() > max_velocity 
        { if current_velocity > 0.0 
            { axis.previous_setpoint + max_velocity } 
            else { axis.previous_setpoint - max_velocity } 
        }
        else { current_setpoint };

    axis.previous_setpoint = new_setpoint;

    new_setpoint
}


fn apply_feeedforward_limit(
    value: f32,
    current_setpoint: f32,
    k_rate_p : f32,
    max_rate_limit: f32) -> f32 {

        if value * current_setpoint > 0.0 {
            if current_setpoint.abs() <= max_rate_limit
            { constrain_f(value, (-max_rate_limit -
                    current_setpoint) * k_rate_p,
                    (max_rate_limit - current_setpoint) * k_rate_p)}
            else {
                0.0
            }
        }
        else {
            0.0
        }
    }

fn init_lpf1(cyclic_axis: &mut CyclicAxis, cutoff_freq: f32) {

    filters::adjust_pt1_gain(cyclic_axis.dterm_lpf1, cutoff_freq);
}

fn dyn_lpf_cutoff_freq(throttle: f32, dyn_lpf_min: f32, dyn_lpf_max: f32, expo: f32) -> f32 {
    let expof = expo / 10.0;
    let curve = throttle * (1.0 - throttle) * expof + throttle;

    (dyn_lpf_max - dyn_lpf_min) * curve + dyn_lpf_min
}


fn constrain_output(demand: f32, limit: f32) -> f32 {

    constrain_f(demand, -limit, limit) / OUTPUT_SCALING
}
