/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>

namespace hf {

    class Quaternion {

        public:

            float w;
            float x;
            float y;
            float z;

            Quaternion() : w(1), x(0), y(0), z(0) {}

            Quaternion(
                    const float w,
                    const float x,
                    const float y,
                    const float z) 
                : w(w), x(x), y(y), z(z) {}

            Quaternion& operator=(const Quaternion& other) = default;

            Quaternion operator/(const float d) const
            {
                return Quaternion(w/d, x/d, y/d, z/d);
            }

            static auto l2norm(const Quaternion& q) -> float
            {
                return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
            }

#if 0
            Quaternion operator+(const Quaternion& other) const
            {
                return Quaternion(w+other.w, x+other.x, y+other.y, z+other.z);
            }

            Quaternion operator-(const Quaternion& other) const
            {
                return Quaternion(w-other.w, x-other.x, y-other.y, z-other.z);
            }

            Quaternion operator*(const Quaternion& other) const
            {
                return Quaternion(w*other.w, x*other.x, y*other.y, z*other.z);
            }

            Quaternion operator*(const float v) const
            {
                return Quaternion(w*v, x*v, y*v, z*v);
            }

#endif
    };

}
