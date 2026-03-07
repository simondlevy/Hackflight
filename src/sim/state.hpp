#pragma once

/*
 *   Copyright (C) 2026 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, in version 3.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <sim/pose.h>

namespace hf {

    // From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
    // We use ENU coordinates based on 
    // https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
    // Position in meters, velocity in meters/second, angles in degrees,
    // angular velocity in degrees/second.
    class SimState {

        public:

            double x;       // positive forward
            double dx;      // positive forward
            double y;       // positive rightward
            double dy;      // positive rightward
            double z;       // positive upward
            double dz;      // positive upward
            double phi;     // positive roll right
            double dphi;    // positive roll right
            double theta;   // positive nose down
            double dtheta;  // positive nose down
            double psi;     // positive nose right
            double dpsi;    // positive nose right

            SimState() 
                : x(0), dx(0), y(0), dy(0), z(0), dz(0), phi(0),
                dphi(0), theta(0), dtheta(0), psi(0), dpsi(0) {}

            SimState(const pose_t & p) 
                : x(p.x), dx(0), y(p.y), dy(0), z(p.z), dz(0),
                phi(p.phi), dphi(0), theta(p.theta), dtheta(0),
                psi(p.psi), dpsi(0) {}

            SimState(
                    const double x, const double dx,
                    const double y, const double dy,
                    const double z, const double dz,
                    const double phi, const double dphi,
                    const double theta, const double dtheta,
                    const double psi, const double dpsi)
                : x(x), dx(dx), y(y), dy(dy), z(z), dz(dz), phi(phi),
                dphi(dphi), theta(theta), dtheta(dtheta), psi(psi), dpsi(dpsi) {}
    };
}
