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

            static void quat2euler(
                    const float qw, const float qx, const float qy, const float qz,
                    float & phi, float & theta, float & psi)
            {
                // We swap the X and Y axes and negate Y for nose-down positive.
                phi = RAD2DEG * asin((-2) * (qx*qz - qw*qy));

                theta = -RAD2DEG * atan2((2 * (qy*qz + qw*qx)),
                        (qw*qw - qx*qx - qy*qy + qz*qz));

                // Negate for nose-right positive
                psi = -RAD2DEG * atan2((2 * (qx*qy + qw*qz)),
                        (qw*qw + qx*qx - qy*qy - qz*qz));
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
    };

}
