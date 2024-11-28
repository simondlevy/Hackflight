/* 
 *  Get vehicle state directly from Dynamics object
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

#include <hackflight.hpp>
#include <sim/dynamics.hpp>

hf::state_t estimate_state(
        const hf::Dynamics & dynamics,
        const float pid_rate)
{
    (void)pid_rate;

    return hf::state_t {

            0,                                  // inertial frame x, unused

            dynamics.x2 * cos(dynamics.x11) -        // body frame dx
                dynamics.x4 * sin(dynamics.x11),

            0,                              // inertial frame y, unused

            -(dynamics.x2 * sin(dynamics.x11) +     // body frame dy
                    dynamics.x4 * cos(dynamics.x11)),

            dynamics.x5,                             // inertial frame z

            dynamics.x6,                             // inertial frame dz

            hf::Utils::RAD2DEG* dynamics.x7,         // phi

            hf::Utils::RAD2DEG* dynamics.x8,         // dphi

            hf::Utils::RAD2DEG* dynamics.x9,         // theta

            hf::Utils::RAD2DEG* dynamics.x10,        // dtheta

            hf::Utils::RAD2DEG* dynamics.x11,        // psi

            hf::Utils::RAD2DEG* dynamics.x12         // dpsi
    };
}
