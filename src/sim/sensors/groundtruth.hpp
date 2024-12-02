/* 
 *  A ground-truth "sensor" that gets vehicle state directly from Dynamics
 *  object
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
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

#pragma once

#include <hackflight.hpp>
#include <sim/dynamics.hpp>

namespace hf {

    class GroundTruth {

        public:

            static state_t read(const Dynamics & d)
            {
                state_t state = {};

                state.dx = d.x2 * cos(d.x11) - d.x4 * sin(d.x11);

                state.dy = -(d.x2 * sin(d.x11) + d.x4 * cos(d.x11));

                state.z = d.x5;                     

                state.dz = d.x6;                   

                state.phi = hf::Utils::RAD2DEG* d.x7; 

                state.dphi = hf::Utils::RAD2DEG* d.x8; 

                state.theta = hf::Utils::RAD2DEG* d.x9; 

                state.dtheta = hf::Utils::RAD2DEG* d.x10;

                state.psi = hf::Utils::RAD2DEG* d.x11;   

                state.dpsi = hf::Utils::RAD2DEG* d.x12;   

                return state;
            }

    };
}
