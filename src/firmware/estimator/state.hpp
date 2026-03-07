/**
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

namespace hf {

    class EstimatedState { 

        public:

            float dx;      // positive forward
            float dy;      // positive rightward
            float z;       // positive upward
            float dz;      // positive upward
            float phi;     // positive roll right
            float theta;   // positive nose down
            float psi;     // positive nose right

            EstimatedState() = default;

            EstimatedState
                (const float dx,
                 const float dy,
                 const float z,
                 const float dz,
                 const float phi,
                 const float theta,
                 const float psi)
                : 
                    dx(dx),
                    dy(dy),
                    z(z),
                    dz(dz),
                    phi(phi),
                    theta(theta),
                    psi(psi) {}

            EstimatedState& operator=(const EstimatedState& other) = default;
 
    };
}

