/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

use crate::datatypes::Demands;
use crate::datatypes::VehicleState;
use crate::filters::Pt1Filter;
use crate::filters::Pt2Filter;
use crate::utils::constrain;
use crate::utils::max;
use crate::utils::min;

use micromath::F32Ext;

const K_RATE_P : f32 = 1.441305;     
const K_RATE_I : f32 = 48.8762;      
const K_RATE_D : f32 = 0.021160;     
const K_RATE_F : f32 = 0.0165048;    
const K_LEVEL_P : f32 = 0.; // 3.0

// minimum of 5ms between updates
const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u32 = 5000; 

const DYN_LPF_THROTTLE_STEPS : u32 = 100;

// Full iterm suppression in setpoint mode at high-passed setpoint rate
// > 40deg/sec
const ITERM_RELAX_SETPOINT_THRESHOLD : f32= 40.;
const ITERM_RELAX_CUTOFF: f32 = 15.;

const DTERM_LPF1_DYN_MIN_HZ: f32 = 75.;
const DTERM_LPF1_DYN_MAX_HZ: f32 = 150.;
const DTERM_LPF2_HZ: f32 = 150.;

const YAW_LOWPASS_HZ: f32 = 100.0;

const ITERM_WINDUP_POINT_PERCENT: f32 = 85.;        

const D_MIN: f32 = 30.;
const D_MIN_GAIN: f32 = 37.;
const D_MIN_ADVANCE: f32 = 20.;

const FEEDFORWARD_MAX_RATE_LIMIT: f32 = 90.;

const DYN_LPF_CURVE_EXPO: f32 = 5.;

// PT2 lowpass input cutoff to peak D around propwash frequencies
const D_MIN_RANGE_HZ: f32 = 85.;  

// PT2 lowpass cutoff to smooth the boost effect
const D_MIN_LOWPASS_HZ: f32 = 35.;  
const D_MIN_GAIN_FACTOR: f32 = 0.00008;
const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;

const RATE_ACCEL_LIMIT: u16 = 0;
const YAW_RATE_ACCEL_LIMIT: f32 = 0.;
const ITERM_LIMIT: f32 = 400.;

const LEVEL_ANGLE_LIMIT: f32 = 45.;

const OUTPUT_SCALING: f32 = 1000.;
const LIMIT_YAW: f32  = 400.;
const LIMIT: f32 = 500.;

const MAX_VELOCITY_CYCLIC: f32 = (RATE_ACCEL_LIMIT as f32) * 100. * DT;


const MAX_VELOCITY_YAW: f32 = YAW_RATE_ACCEL_LIMIT * 100. * DT; 

const FREQ_HZ: f32 = 8000.;
const PERIOD: f32 = 1000000. / FREQ_HZ;
const DT: f32 = (PERIOD as f32) * 1e-6;


 #[derive(Clone,Copy)]
struct Axis {

    previous_setpoint : f32,
    integral : f32
}

impl Axis {

    fn acceleration_limit(&mut self, current_setpoint: f32, max_velocity: f32) -> f32 {

        let current_velocity = current_setpoint - self.previous_setpoint;

        let new_setpoint = 
            if current_velocity.abs() > max_velocity {
                if current_velocity > 0.  {
                    self.previous_setpoint + max_velocity 
                }
                else  {
                    self.previous_setpoint - max_velocity
                }
            }
            else {
                current_setpoint
            };

        self.previous_setpoint = new_setpoint;

        new_setpoint
    }
}

#[derive(Copy,Clone)]
struct CyclicAxis {

    axis: Axis,
    dterm_lpf1 : Pt1Filter,
    dterm_lpf2 : Pt1Filter,
    d_min_lpf: Pt2Filter,
    d_min_range: Pt2Filter,
    windup_lpf: Pt1Filter,
    previous_dterm: f32
}

impl CyclicAxis {

    fn init(&mut self) {

        self.dterm_lpf1.init();
        self.dterm_lpf2.init();
        self.d_min_lpf.init();
        self.d_min_range.init();
        self.windup_lpf.init();
    }

    fn apply_iterm_relax(
        &mut self,
        iterm: f32,
        current_setpoint: f32,
        iterm_error_rate: f32) -> f32 {

        let setpoint_lpf = self.windup_lpf.apply(current_setpoint);

        let setpoint_hpf = (current_setpoint - setpoint_lpf).abs();

        let iterm_relax_factor =
            max(0., 1. - setpoint_hpf / ITERM_RELAX_SETPOINT_THRESHOLD);

        let is_decreasing_i =
            ((iterm > 0.) && (iterm_error_rate < 0.)) ||
            ((iterm < 0.) && (iterm_error_rate > 0.));

        iterm_error_rate * (if !is_decreasing_i { iterm_relax_factor } else { 1. })
    }

    fn compute_d_min_factor(
        mut self, d_min_percent: f32, demand_delta: f32, delta: f32) -> f32
    {
        let d_min_gyro_gain = D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;

        let d_min_gyro_factor = (self.d_min_range.apply(delta)).abs() * d_min_gyro_gain;

        let d_min_setpoint_gain =
            D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR *
            D_MIN_ADVANCE * FREQ_HZ /
            (100. * D_MIN_LOWPASS_HZ);

        let d_min_setpoint_factor = demand_delta.abs() * d_min_setpoint_gain;

        let d_min_factor = d_min_percent + (1. - d_min_percent) *
            max(d_min_gyro_factor, d_min_setpoint_factor);

        let d_min_factor_filtered = self.d_min_lpf.apply(d_min_factor);

        min(d_min_factor_filtered, 1.)
    }

    fn init_lpf1(&mut self, cutoff_freq: f32)
    {
        self.dterm_lpf1.compute_gain(cutoff_freq);
    }
}

#[derive(Copy,Clone)]
pub struct AnglePid {

    pub k_rate_p: f32,
    pub k_rate_i: f32,
    pub k_rate_d: f32,
    pub k_rate_f: f32,
    pub k_level_p: f32,
    pub usec_prev: u32,
    pub dyn_lpf_previous_quantized_throttle: i32,  
    roll : CyclicAxis,
    pitch : CyclicAxis,
    yaw: Axis,
    pterm_yaw_lpf: Pt1Filter,
}

const INIT_AXIS : Axis = Axis {

    previous_setpoint : 0.,
    integral : 0.
};

const INIT_CYCLIC_AXIS : CyclicAxis = CyclicAxis {

    axis: INIT_AXIS,

    dterm_lpf1: Pt1Filter {
        state: 0.,
        dt: DT,
        cutoff: DTERM_LPF1_DYN_MIN_HZ,
        k: 0.
    },

    dterm_lpf2: Pt1Filter {
        state: 0.,
        dt: DT,
        cutoff: DTERM_LPF2_HZ,
        k: 0.
    },

    d_min_lpf: Pt2Filter {
        state: 0.,
        state1: 0.,
        dt: 0.001,
        cutoff: D_MIN_LOWPASS_HZ,
        k: 0.
    },

    d_min_range: Pt2Filter {
        state: 0.,
        state1: 0.,
        dt: 0.001,
        cutoff: D_MIN_RANGE_HZ,
        k: 0.
    },

    windup_lpf: Pt1Filter {
        state: 0.,
        dt: DT,
        cutoff: ITERM_RELAX_CUTOFF,
        k: 0.
    },

    previous_dterm: 0.
};

pub const INIT : AnglePid = AnglePid {

    k_rate_p: K_RATE_P,
    k_rate_i: K_RATE_I,
    k_rate_d: K_RATE_D,
    k_rate_f: K_RATE_F,
    k_level_p: K_LEVEL_P,

    usec_prev: 0,

    dyn_lpf_previous_quantized_throttle: 0,

    roll: INIT_CYCLIC_AXIS,

    pitch: INIT_CYCLIC_AXIS,

    yaw : INIT_AXIS,

    pterm_yaw_lpf: Pt1Filter {
        state: 0.,
        dt: DT,
        cutoff: YAW_LOWPASS_HZ,
        k: 0.
    }
};

// [-1,+1] => [-670,+670] with nonlinearity
fn rescale(command : f32) -> f32 {

    let ctr = 0.104;

    let expof = command * command.abs();
    let angle_rate = command * ctr + (1. - ctr) * expof;

    670. * angle_rate
}

impl AnglePid {

    pub fn init(&mut self) {

        self.roll.init();
        self.pitch.init();
        self.pterm_yaw_lpf.init();
    }

    pub fn get_demands(
        &mut self,
        usec: u32,
        demands: Demands,
        vstate: VehicleState,
        reset: bool) -> Demands {

        let dusec = usec - self.usec_prev;

        self.usec_prev = usec;

        let roll_demand = rescale(demands.roll);
        let pitch_demand = rescale(demands.pitch);
        let yaw_demand = rescale(demands.yaw);

        let roll =
            self.update_cyclic(roll_demand, vstate.phi, vstate.dphi, &mut self.roll);

        let pitch =
            self.update_cyclic(pitch_demand, vstate.theta, vstate.dtheta, &mut self.pitch);

        let yaw = self.update_yaw(yaw_demand, vstate.dpsi);

        if reset {
            self.roll.axis.integral = 0.;
            self.pitch.axis.integral = 0.;
            self.yaw.integral = 0.;
        }

        if dusec >= DYN_LPF_THROTTLE_UPDATE_DELAY_US {

            // quantize the throttle reduce the number of filter updates
            let quantized_throttle =
                (demands.throttle * (DYN_LPF_THROTTLE_STEPS as f32)).round() as i32; 

            if quantized_throttle != self.dyn_lpf_previous_quantized_throttle {

                // scale the quantized value back to the throttle range so the
                // filter cutoff steps are repeatable
                let dyn_lpf_throttle =
                    (quantized_throttle as f32) / (DYN_LPF_THROTTLE_STEPS as f32);
                self.pid_dyn_lpf_dterm_update(dyn_lpf_throttle);
                self.dyn_lpf_previous_quantized_throttle = quantized_throttle;
            }
        }

        Demands { 
            throttle: demands.throttle,
            roll: constrain_output(roll, LIMIT),
            pitch: constrain_output(pitch, LIMIT),
            yaw: -constrain_output(yaw, LIMIT_YAW)
        }
    }

    fn pid_dyn_lpf_dterm_update(mut self, throttle: f32)
    {
        let dyn_lpf_min = DTERM_LPF1_DYN_MIN_HZ;
        let dyn_lpf_max = DTERM_LPF1_DYN_MAX_HZ;

        let cutoff_freq =
            dyn_lpf_cutoff_freq(throttle, dyn_lpf_min, dyn_lpf_max, DYN_LPF_CURVE_EXPO);

        self.roll.init_lpf1(cutoff_freq);
        self.pitch.init_lpf1(cutoff_freq);
    }

    fn update_cyclic(
        self,
        demand: f32,
        angle: f32,
        angvel: f32,
        cyclic_axis : &mut CyclicAxis) -> f32 {

        let current_setpoint = 
            if MAX_VELOCITY_CYCLIC > 0. {
                cyclic_axis.axis.acceleration_limit(demand, MAX_VELOCITY_CYCLIC)
            }
            else {
                demand
            };

        let new_setpoint = self.level_pid(current_setpoint, angle);

        // -----calculate error rate
        let error_rate = new_setpoint - angvel;

        let iterm_error_rate = cyclic_axis.apply_iterm_relax(
            cyclic_axis.axis.integral,
            new_setpoint,
            error_rate);

        // -----calculate P component
        let p = self.k_rate_p * error_rate;

        // -----calculate I component
        cyclic_axis.axis.integral =
            constrain(cyclic_axis.axis.integral + self.k_rate_i * DT * iterm_error_rate,
                      -ITERM_LIMIT,
                      ITERM_LIMIT);

        // -----calculate D component
        let dterm = cyclic_axis.dterm_lpf2.apply(cyclic_axis.dterm_lpf1.apply(angvel));

        let d =
            if self.k_rate_d > 0. {
                self.compute_derivative(*cyclic_axis, 0., dterm) 
            }
            else {
                0.
            };

        cyclic_axis.previous_dterm = dterm;

        // -----calculate feedforward component
        let f =
            if self.k_rate_f > 0.  { 
                self.compute_feedforward(new_setpoint, 670., 0.) 
            }
            else {
                0.
            };

        p + cyclic_axis.axis.integral + d + f
    }

    fn update_yaw(mut self, demand: f32, angvel: f32) -> f32 {

        // gradually scale back integration when above windup point
        let iterm_windup_point_inv =
            1. / (1. - (ITERM_WINDUP_POINT_PERCENT / 100.));

        let dyn_ci = DT * 
            (if iterm_windup_point_inv > 1. {
                constrain(iterm_windup_point_inv, 0., 1.)
            }
            else {
                1.
            });

        let max_velocity = MAX_VELOCITY_YAW;

        let current_setpoint = 
            if max_velocity > 0. {
                self.yaw.acceleration_limit(demand, max_velocity)
            } else {
                demand
            };

        let error_rate = current_setpoint - angvel;

        // -----calculate P component
        let p = self.pterm_yaw_lpf.apply(self.k_rate_p * error_rate);

        // -----calculate I component, constraining windup
        self.yaw.integral = constrain(
            self.yaw.integral + (self.k_rate_i * dyn_ci) * error_rate,
            -ITERM_LIMIT,
            ITERM_LIMIT);

        p + self.yaw.integral
    }

    fn compute_feedforward(
        self, current_setpoint: f32, feedforward_max_rate: f32, demand_delta: f32) -> f32 {

        // halve feedforward in Level mode since stick sensitivity is
        // weaker by about half transition now calculated in
        // feedforward.c when new RC data arrives 
        let feedforward = self.k_rate_f * demand_delta * FREQ_HZ;

        let feedforward_max_rate_limit =
            feedforward_max_rate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01;

        if feedforward_max_rate_limit != 0. {
            self.apply_feedforward_limit(
                feedforward, current_setpoint, feedforward_max_rate_limit) 
        } 
        else {
            feedforward
        }
    }

    fn apply_feedforward_limit(
        self, value: f32, current_setpoint: f32, max_rate_limit: f32) -> f32 { 

        if value * current_setpoint > 0. {
            if current_setpoint.abs() < max_rate_limit {
                constrain(value, 
                          (-max_rate_limit - current_setpoint) * self.k_rate_p,
                          (max_rate_limit - current_setpoint) * self.k_rate_p)
            }
            else {
                0.
            }
        }
        else {
            0.
        }
    }

    fn compute_derivative(
        self, cyclic_axis: CyclicAxis, demand_delta: f32, dterm: f32) -> f32
    {
        // Divide rate change by dT to get differential (ie dr/dt).
        // dT is fixed and calculated from the target PID loop time
        // This is done to avoid DTerm spikes that occur with
        // dynamically calculated deltaT whenever another task causes
        // the PID loop execution to be delayed.
        let delta = -(dterm - cyclic_axis.previous_dterm) * (FREQ_HZ as f32);

        let pretpad = self.k_rate_d * delta;

        let d_min_percent = 
            if D_MIN > 0. && D_MIN < self.k_rate_d {
                D_MIN / self.k_rate_d 
            } 
            else {
                0.
            };

        let d_min_factor =
            if d_min_percent > 0. {
                cyclic_axis.compute_d_min_factor(d_min_percent, demand_delta, delta) 
            }
            else {
                1.
            };

        // Apply the d_min_factor
        pretpad * d_min_factor
    }

    fn  level_pid(self, current_setpoint: f32, current_angle: f32) -> f32
    {
        // calculate error angle and limit the angle to the max inclination
        // rcDeflection in [-1.0, 1.0]

        let angle = constrain(LEVEL_ANGLE_LIMIT * current_setpoint,
                              -LEVEL_ANGLE_LIMIT,
                              LEVEL_ANGLE_LIMIT);

        let angle_error = angle - (current_angle / 10.);

        if self.k_level_p > 0. {
            angle_error * self.k_level_p }
        else {
            current_setpoint
        }
    }

} // impl AnglePid

fn dyn_lpf_cutoff_freq(
     throttle: f32, dyn_lpf_min: f32, dyn_lpf_max: f32, expo: f32)  -> f32 {

    let expof = expo / 10.;
    let curve = throttle * (1. - throttle) * expof + throttle;
    (dyn_lpf_max - dyn_lpf_min) * curve + dyn_lpf_min
}

fn constrain_output(demand: f32, limit: f32) -> f32 {

    constrain(demand, -limit, limit) / OUTPUT_SCALING
}
