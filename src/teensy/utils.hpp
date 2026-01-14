/*
   Utilities for Hackflight

   Copyright (C) 2024 Simon D. Levy

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

#include <math.h>

#include <datatypes.h>

class Utils {

    public:

        static constexpr float RAD2DEG = 180.0f / M_PI;

        static void quat2euler(const axis4_t & q, axis3_t &a)
        {
            a.x = RAD2DEG * atan2f(q.w*q.x + q.y*q.z,
                    0.5f - q.x*q.x - q.y*q.y);

            a.y = RAD2DEG * asinf(2 * (q.x*q.z - q.w*q.y));

            a.z = RAD2DEG * atan2f(q.x*q.y + q.w*q.z,
                    0.5f - q.y*q.y - q.z*q.z);
        }
};
