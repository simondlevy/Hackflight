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

                state.dx = d.state.dx * cos(d.state.psi) - d.state.dy * sin(d.state.psi);

                state.dy = -(d.state.dx * sin(d.state.psi) + d.state.dy * cos(d.state.psi));

                state.z = d.state.z;                     

                state.dz = d.state.dz;                   

                state.phi = hf::Utils::RAD2DEG* d.state.phi; 

                state.dphi = hf::Utils::RAD2DEG* d.state.dphi; 

                state.theta = hf::Utils::RAD2DEG* d.state.theta; 

                state.dtheta = hf::Utils::RAD2DEG* d.state.dtheta;

                state.psi = hf::Utils::RAD2DEG* d.state.psi;   

                state.dpsi = hf::Utils::RAD2DEG* d.state.dpsi;   

                return state;
            }

    };
}
