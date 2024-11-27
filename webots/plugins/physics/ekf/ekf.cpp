/* 
 * Custom physics plugin for Hackflight simulator using Estimated Kalman Filter for
 * state and standard C++ PID controllers
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


#include <sim/standard_controllers.hpp>
#include <estimators/ekf.hpp>
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
hf::state_t estimate_state(const hf::Dynamics & dynamics)
{
    const auto gyro = hf::Gyrometer::read(dynamics);

    const auto accel = hf::Accelerometer::read(dynamics);

    _ekf.accumulate_gyro(gyro);

    _ekf.accumulate_accel(accel);

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
