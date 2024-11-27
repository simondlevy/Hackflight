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

hf::state_t estimate_state(const hf::Dynamics & dynamics)
{
    const auto gyro = hf::Gyrometer::read(dynamics);

    return hf::state_t {

            0,                                  // inertial frame x, unused

            dynamics._x2 * cos(dynamics._x11) -        // body frame dx
                dynamics._x4 * sin(dynamics._x11),

            0,                              // inertial frame y, unused

            -(dynamics._x2 * sin(dynamics._x11) +     // body frame dy
                    dynamics._x4 * cos(dynamics._x11)),

            dynamics._x5,                             // inertial frame z

            dynamics._x6,                             // inertial frame dz

            hf::Utils::RAD2DEG* dynamics._x7,         // phi

            gyro.x,         // dphi

            hf::Utils::RAD2DEG* dynamics._x9,         // theta

            gyro.y,        // dtheta

            hf::Utils::RAD2DEG* dynamics._x11,        // psi

            gyro.z         // dpsi
    };
}
