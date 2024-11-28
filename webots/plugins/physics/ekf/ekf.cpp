/* 
 * Custom physics plugin for Hackflight simulator using Estimated Kalman Filter
 * for state and standard C++ PID controllers
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <stdio.h>

#include <hackflight.hpp>
#include <estimators/ekf.hpp>
#include <sim/standard_controllers.hpp>
#include <sim/sensors/accelerometer.hpp>
#include <sim/sensors/gyrometer.hpp>
#include <sim/sensors/optical_flow.hpp>
#include <sim/sensors/rangefinder.hpp>
#include <utils.hpp>

// All rates in Hz, as measured in Crazyflie state estimator
static const float PREDICTION_RATE = 100;
static const float OPTICAL_FLOW_RATE = 100;
static const float RANGEFINDER_RATE = 25;

static hf::EKF _ekf;

static FILE * _logfp;

void setup_controllers()
{
    _ekf.initialize();

    _logfp = fopen("log.csv", "w");
}

// Called by webots_physics_step()
hf::state_t estimate_state(
        const hf::Dynamics & dynamics, const float pid_rate)
{
    // Gyro and accel accumulation run at same 1000Hz rate as control loop
    const auto gyro = hf::Gyrometer::read(dynamics);
    _ekf.accumulate_gyro(gyro);
    _ekf.accumulate_accel(hf::Accelerometer::read(dynamics));

    static uint32_t _prediction_count;
    if (_prediction_count++ == (uint32_t)(pid_rate/PREDICTION_RATE)) {
        _ekf.predict(1 / PREDICTION_RATE);
        _prediction_count = 0;
    }

    static uint32_t _flow_count;
    if (_flow_count++ == (uint32_t)(pid_rate/OPTICAL_FLOW_RATE)) {
        _ekf.update_with_flow(1/OPTICAL_FLOW_RATE,
                hf::OpticalFlow::read(dynamics));
        _flow_count = 0;
    }

    static uint32_t _range_count;
    if (_range_count++ == (uint32_t)(pid_rate/RANGEFINDER_RATE)) {
        _ekf.update_with_range(hf::Rangefinder::read(dynamics));
        _range_count = 0;
    }

    _ekf.finalize();

    hf::axis4_t quat_est = {};
    hf::axis2_t dxdy_est = {};
    float z_est = 0;
    float dz_est = 0;

    _ekf.get_vehicle_state(quat_est, dxdy_est, z_est, dz_est);

    hf::axis3_t euler_est = {};
    hf::Utils::quat2euler(quat_est, euler_est);

    static hf::state_t _state;

    // dx/dt, body frame
    _state.dx = dynamics.x2 * cos(dynamics.x11) -        
        dynamics.x4 * sin(dynamics.x11);

    // dx/dt, body frame
    _state.dy  = -(dynamics.x2 * sin(dynamics.x11) +    
            dynamics.x4 * cos(dynamics.x11));

    // z, inertial frame
    _state.z = z_est;

    // dz/dt, inertial frame
    _state.dz = dz_est;

    // phi
    _state.phi = euler_est.x;

    // dphi/dt, directly from gyro
    _state.dphi = gyro.x;

    // theta (note negation)
    _state.theta = -euler_est.y;

    // dtheta/dt, directly from gyro
    _state.dtheta = gyro.y;

    // psi, directly from gyro
    _state.psi = euler_est.z;

    // dpsi/dt
    _state.dpsi = gyro.z;

    return _state;
}
