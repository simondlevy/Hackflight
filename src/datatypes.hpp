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

    class Vec3 {

        public:

            float x;
            float y;
            float z;

            Vec3() = default;

            Vec3(const float x, const float y, const float z) 
                : x(x), y(y), z(z) {}

            Vec3& operator=(const Vec3& other) = default;

            Vec3 operator+(const Vec3& other) const
            {
                return Vec3(x+other.x, y+other.y, z+other.z);
            }

            Vec3 operator-(const Vec3& other) const
            {
                return Vec3(x-other.x, y-other.y, z-other.z);
            }

            Vec3 operator*(const Vec3& other) const
            {
                return Vec3(x*other.x, y*other.y, z*other.z);
            }

            Vec3 operator/(const float d) const
            {
                return Vec3(x/d, y/d, z/d);
            }
      };

    class Vec4 {

        public:

            float w;
            float x;
            float y;
            float z;

            Vec4() = default;

            Vec4
                (const float w, const float x, const float y, const float z) 
                : w(w), x(x), y(y), z(z) {}

            Vec4& operator=(const Vec4& other) = default;
    };
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

    //typedef void (*mixFun_t)(const Setpoint & setpoint, float motorvals[]);
}
