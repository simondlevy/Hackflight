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

#include <stdint.h>

namespace hf {

    class VehicleState {

        public:

            float dx;      // positive forward
            float dy;      // positive rightward
            float z;       // positive upward
            float dz;      // positive upward
            float phi;     // positive roll right
            float dphi;    // positive roll right
            float theta;   // positive nose down
            float dtheta;  // positive nose down
            float psi;     // positive nose right
            float dpsi;    // positive nose right

            VehicleState() = default;

            VehicleState
                (const float dx,
                 const float dy,
                 const float z,
                 const float dz,
                 const float phi,
                 const float dphi,
                 const float theta,
                 const float dtheta,
                 const float psi,
                 const float dpsi) 
                : 
                    dx(dx),
                    dy(dy),
                    z(z),
                    dz(dz),
                    phi(phi),
                    dphi(dphi),
                    theta(theta),
                    dtheta(dtheta),
                    psi(psi),
                    dpsi(dpsi) {}

            VehicleState& operator=(const VehicleState& other) = default;
    };

    typedef enum {

        MODE_IDLE,
        MODE_ARMED,
        MODE_HOVERING,
        MODE_AUTONOMOUS,
        MODE_LANDING,
        MODE_PANIC

    } mode_e;


    class Setpoint {

        public:

            float thrust;  // positve upward
            float roll;    // positive roll right
            float pitch;   // positive nose down
            float yaw;     // positive nose right

            Setpoint() = default;

            Setpoint
                (const float t, const float r, const float p, const float y) 
                : thrust(t), roll(r), pitch(p), yaw(y) {}

            Setpoint& operator=(const Setpoint& other) = default;
    };
}
