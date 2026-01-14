/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <datatypes.h>

class Num {

    public:

        static constexpr float RAD2DEG = 180.0f / M_PI;
        static constexpr float DEG2RAD = M_PI / 180.0f;

        static float fconstrain(const float val, const float maxabs)
        {
            return val < -maxabs ? -maxabs : val > maxabs ? maxabs : val;
        }

        static float fconstrain(
                float value, const float minVal, const float maxVal)
        {
            return fminf(maxVal, fmaxf(minVal,value));
        }

        static void euler2quat(const axis3_t & a, axis4_t & q)
        {
            // Abbreviations for the various angular functions

            const auto cr = (float)cos(a.x / 2);
            const auto sr = (float)sin(a.x / 2);
            const auto cp = (float)cos(a.y / 2);
            const auto sp = (float)sin(a.y / 2);
            const auto cy = (float)cos(a.z / 2);
            const auto sy = (float)sin(a.z / 2);

            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
        }

        static void quat2euler(const axis4_t & q, axis3_t & a)
        {
       }

        static float cap_angle(float angle) 
        {
            float result = angle;

            while (result > 180.0f) {
                result -= 360.0f;
            }

            while (result < -180.0f) {
                result += 360.0f;
            }

            return result;
        }

        static float rescale(
                const float value,
                const float oldmin, 
                const float oldmax, 
                const float newmin, 
                const float newmax) 
        {
            return (value - oldmin) / (oldmax - oldmin) * 
                (newmax - newmin) + newmin;
        }

};
