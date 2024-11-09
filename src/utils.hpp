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
#include <hackflight.hpp>

#ifndef M_PI
#define M_PI 3.1415928f
#endif

namespace hf {

    class Utils {

        public:

            static constexpr float DEG2RAD = M_PI / 180.0f;
            static constexpr float RAD2DEG = 180.0f / M_PI;
            static constexpr float GS2MSS = 9.81;

            static void quat2euler(const quaternion_t q, axis3_t &a)
            {
                a.x = RAD2DEG * atan2(q.w*q.x + q.y*q.z,
                        0.5f - q.x*q.x - q.y*q.y);

                a.y = RAD2DEG * asin(2 * (q.x*q.z - q.w*q.y));

                a.z = RAD2DEG * atan2(q.x*q.y + q.w*q.z,
                        0.5f - q.y*q.y - q.z*q.z);
            }

            static float fmax(const float val, const float maxval)
            {
                return val > maxval ? maxval : val;
            }

            static float fmin(const float val, const float maxval)
            {
                return val < maxval ? maxval : val;
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

            static float square(const float x)
            {
                return x * x;
            }

            static float fconstrain(const float val, const float maxabs)
            {
                return val < -maxabs ? -maxabs : val > maxabs ? maxabs : val;
            }

            static uint8_t u8constrain(
                    const uint8_t val, const uint8_t minval, const uint8_t maxval)
            {
                return val < minval ? minval : val > maxval ? maxval : val;
            }

            static bool in_deadband(const float val, const float band)
            {
                return fabs(val) < band;
            }

            // https://en.wikipedia.org/wiki/Rotation_matrix
            static void angles2rotation(const axis3_t & a, float r[3][3]) 
            {
                const auto alpha = a.z;
                const auto beta = a.y;
                const auto gamma = a.x;

                r[0][0] = cos(beta) * cos(gamma);
                r[0][1] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
                r[0][2] = cos(alpha) * sin(beta) + sin(alpha) * sin(gamma);
                r[1][0] = cos(beta) * sin(gamma);
                r[1][1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
                r[1][2] = cos(alpha) * sin(beta) - sin(alpha) * cos(gamma);
                r[2][0] = -sin(beta);
                r[2][1] = sin(alpha) * cos(beta);
                r[2][2] = cos(alpha) * cos(beta);
            }
    };

}
