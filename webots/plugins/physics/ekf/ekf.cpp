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

// All rates in Hz
static const float PREDICTION_RATE = 100;
static const float OPTICAL_FLOW_RATE = 100;
static const float RANGEFINDER_RATE = 25;

static const float FINALIZE_RATE = 1000;
static const float GYRO_RATE = 1000;
static const float ACCEL_RATE = 1000;

static hf::EKF _ekf;

void setup_controllers()
{
    _ekf.initialize();
}

// Called by webots_physics_step()
hf::state_t estimate_state(
        const hf::Dynamics & dynamics, const float pid_rate)
{
    (void)pid_rate;

    const auto gyro = hf::Gyrometer::read(dynamics);

    const auto accel = hf::Accelerometer::read(dynamics);

    _ekf.accumulate_gyro(gyro);

    _ekf.accumulate_accel(accel);

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
        _ekf.update_with_range(1/RANGEFINDER_RATE);
        _range_count = 0;
    }

    _ekf.finalize();

    hf::axis4_t quat = {};
    hf::axis2_t dxdy = {};
    float z = 0;
    float dz = 0;

    _ekf.get_vehicle_state(quat, dxdy, z, dz);

    static hf::state_t _state;

    // dx/dt, body frame
    _state.dx = dynamics._x2 * cos(dynamics._x11) -        
        dynamics._x4 * sin(dynamics._x11);

    // dx/dt, body frame
    _state.dy  = -(dynamics._x2 * sin(dynamics._x11) +    
            dynamics._x4 * cos(dynamics._x11));

    // z, inertial frame
    _state.z = dynamics._x5;

    // dz/dt, inertial frame
    _state.dz = dynamics._x6;

    // phi
    _state.phi = hf::Utils::RAD2DEG* dynamics._x7;         

    // dphi/dt
    _state.dphi = gyro.x;

    // theta
    _state.theta = hf::Utils::RAD2DEG* dynamics._x9;

    // dtheta/dt
    _state.dtheta = gyro.y;

    // psi
    _state.psi = hf::Utils::RAD2DEG* dynamics._x11;

    // dpsi/dt
    _state.dpsi = gyro.z;

    return _state;
}
