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

use core::f32::consts::PI;
use micromath::F32Ext;

pub mod math;
pub mod filters;

// ImuSensor ==================================================================

struct ImuSensor {

    values: Axes,
    adcf: Axes,
    count: u32,
}

impl ImuSensor {

    pub fn accumulate(&mut self, x: f32, y: f32, z: f32, period: u32) {

        // integrate using trapezium rule to avoid bias
        self.values.x += 0.5 * (self.adcf.x + x) * (period as f32);
        self.values.y += 0.5 * (self.adcf.x + y) * (period as f32);
        self.values.z += 0.5 * (self.adcf.x + z) * (period as f32);

        self.adcf.x = x;
        self.adcf.y = y;
        self.adcf.z = z;

        self.count +=1;
    }

    pub fn get_average(self, period: u32) -> Axes
    {
        let denom = (self.count * period) as f32;

        Axes {
            x: if denom > 0. {self.values.x / denom} else {0.0},
            y: if denom > 0. {self.values.y / denom} else {0.0},
            z: if denom > 0. {self.values.z / denom} else {0.0},

        }
    }

    pub fn reset(&mut self) {

        self.values.x = 0.0;
        self.values.y = 0.0;
        self.values.z = 0.0;
        self.count = 0;
    }

}

// Quaternion =================================================================

struct Quaternion {

    w : f32,
    x : f32,
    y : f32,
    z : f32,

    ix: f32,
    iy: f32,
    iz: f32,
}


impl Quaternion {

    // Adapted from
    //  https://github.com/jremington/MPU-6050-Fusion/blob/main/MPU6050_MahonyIMU.ino
    fn mahony(
        &mut self,
        dt: f32,
        gyro: Axes,
        accel: Axes,
        q_old: Quaternion,
        kp: f32, ki: f32) {

        let mut gx = math::deg2rad(gyro.x);
        let mut gy = math::deg2rad(gyro.y);
        let mut gz = math::deg2rad(gyro.z);

        let mut ax = accel.x;
        let mut ay = accel.y;
        let mut az = accel.z;

        let mut qw = q_old.w;
        let mut qx = q_old.x;
        let mut qy = q_old.y;
        let mut qz = q_old.z;

        let recip_acc_norm = math::sq(ax) + math::sq(ay) + math::sq(az);

        if recip_acc_norm > 0. {

            // Normalise accelerometer (assumed to measure the direction of
            // gravity in body frame)
            let recip_norm = 1. / recip_acc_norm.sqrt();
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            // Estimated direction of gravity in the body frame (factor of
            // two divided out)
            let vx = qx * qz - qw * qy;  //to normalize these terms,
            let vy = qw * qx + qy * qz;
            let vz = qw * qw - 0.5 + qz * qz;

            // Estimated direction of gravity in the body frame (factor of
            // two divided out)
            let vx = qx * qz - qw * qy;  //to normalize these terms,
            let vy = qw * qx + qy * qz;
            let vz = qw * qw - 0.5 + qz * qz;

            // Error is cross product between estimated and measured
            // direction of gravity in body frame (half the actual
            // magnitude)
            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            // Compute and apply to gyro term the integral feedback, if enabled
            if ki > 0. {
                self.ix += ki * ex * dt;  // integral error scaled by Ki
                self.iy += ki * ey * dt;
                self.iz += ki * ez * dt;
                gx += self.ix;  // apply integral feedback
                gy += self.iy;
                gz += self.iz;
            }

            // Apply proportional feedback to gyro term
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        }
        
        // Integrate rate of change of quaternion, q cross gyro term

        let dtnew = 0.5 * dt;

        gx *= dtnew;   
        gy *= dtnew;
        gz *= dtnew;

        let qa = qw;
        let qb = qx;
        let qc = qy;

        qw += -qb * gx - qc * gy - qz * gz;
        qx += qa * gx + qc * gz - qz * gy;
        qy += qa * gy - qb * gz + qz * gx;
        qz += qa * gz + qb * gy - qc * gx;

        // renormalise quaternion
        let recip_norm = 1. /
            (math::sq(qw) + math::sq(qx) + math::sq(qy) + math::sq(qz)).sqrt();

        self.w *= recip_norm;
        self.x *= recip_norm;
        self.y *= recip_norm;
        self.z *= recip_norm;

    } // mahony

} // impl Quaternion

// PidController ==============================================================

const PID_CONTROLLER_FREQ : u32 = 8000;
const PID_CONTROLLER_PERIOD : u32 = 1000000 / PID_CONTROLLER_FREQ;

// Imu ========================================================================

const GYRO_CALIBRATION_DURATION : u32 = 1250000;
const GYRO_CALIBRATION_CYCLES : u32 = GYRO_CALIBRATION_DURATION / PID_CONTROLLER_PERIOD;

struct Calibration {

}

struct Imu {

    gyro_calibration_cycles_remaining : i32,
    gyro_scale : f32,
    gyro_is_calibrating : bool,
}

impl Imu {

    fn set_gyro_calibration_cycles(&mut self)
    {
        self.gyro_calibration_cycles_remaining = GYRO_CALIBRATION_CYCLES as i32;
    }



}

// Stats ======================================================================

struct Stats {

    old_m : f32,
    new_m : f32,
    old_s : f32,
    new_s : f32,
    n : i32,
}

impl Stats {

    fn variance(self) -> f32 {

        if self.n > 1 { self.new_s / ((self.n - 1) as f32)} else { 0. }
    }

    pub fn stdev_clear(&mut self) {

        self.n = 0
    }
}

// SoftQuatImu ================================================================

struct Fusion {

    time: u32,
    quat: Quaternion,
    rot: Axes
}

struct SoftQuatImu {

    fusion_prev: Fusion,
    gyro_accum : ImuSensor,
    accel_scale : f32,
    accel_axes : Axes,
    short_period : i32,

    accel_filter_x : filters::Pt2Filter,
    accel_filter_y : filters::Pt2Filter,
    accel_filter_z : filters::Pt2Filter,

    imu : Imu,
}

impl SoftQuatImu {

    fn begin(&mut self, clock_speed: u32) {

        const SHORT_THRESHOLD : i32 = 82;

        self.short_period = (clock_speed as i32) / 1000000 * SHORT_THRESHOLD;

        self.imu.set_gyro_calibration_cycles();
    }


    fn accel_filter_init() -> filters::Pt2Filter {

        filters::make_pt2(1. / 1000., 10.)

    }
}
