/* 
 * Custom physics plugin for Hackflight simulator using Madgwick filter for
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

#include <stdio.h>

#include <hackflight.hpp>
#include <estimators/madgwick.hpp>
#include <sim/standard_controllers.hpp>
#include <sim/sensors/accelerometer.hpp>
#include <sim/sensors/gyrometer.hpp>
#include <utils.hpp>

static hf::MadgwickFilter _madgwick;

static FILE * _logfp;

void setup_controllers()
{
    _madgwick.initialize();

    _logfp = fopen("log.csv", "w");
}

// Called by webots_physics_step()
hf::state_t estimate_state(
        const hf::Dynamics & dynamics, const float pid_rate)
{
    const auto gyro = hf::Gyrometer::read(dynamics);
    const auto accel = hf::Accelerometer::read(dynamics);

    static hf::state_t _state;

    // dx/dt, body frame
    _state.dx = dynamics.x2 * cos(dynamics.x11) -        
        dynamics.x4 * sin(dynamics.x11);

    // dx/dt, body frame
    _state.dy  = -(dynamics.x2 * sin(dynamics.x11) +    
            dynamics.x4 * cos(dynamics.x11));

    // z, inertial frame
    _state.z = dynamics.x5;

    // dz/dt, inertial frame
    _state.dz = dynamics.x6;

    // phi
    _state.phi = hf::Utils::RAD2DEG * dynamics.x7;         

    // dphi/dt, directly from gyro
    _state.dphi = gyro.x;

    // theta
    _state.theta = hf::Utils::RAD2DEG* dynamics.x9;

    // dtheta/dt, directly from gyro
    _state.dtheta = gyro.y;

    // psi, directly from gyro
    _state.psi = hf::Utils::RAD2DEG* dynamics.x11;

    // dpsi/dt
    _state.dpsi = gyro.z;

    hf::axis4_t quat = {};
    _madgwick.getQuaternion(1 / pid_rate, gyro, accel, quat);
    //printf("qw=%+3.3f  qx=%+3.3f  qy=%+3.3f  qz=%+3.3f\n",
    //        quat.w, quat.x, quat.y, quat.z);
    hf::axis3_t euler = {};
    hf::Utils::quat2euler(quat, euler);
    fprintf(_logfp, "%f,%f\n", euler.x, _state.phi);

    return _state;
}
