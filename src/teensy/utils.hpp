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

#ifndef M_PI
#define M_PI 3.1415928f
#endif

#include <datatypes.h>

class Utils {

    public:

        static constexpr float RAD2DEG = 180.0f / M_PI;
        static constexpr float G2MSS = 9.81;

        static void quat2euler(const axis4_t & q, axis3_t &a,
                const float ysign=+1, const float zsign=+1)
        {
            a.x = RAD2DEG * atan2(q.w*q.x + q.y*q.z,
                    0.5f - q.x*q.x - q.y*q.y);

            a.y = ysign * RAD2DEG * asin(2 * (q.x*q.z - q.w*q.y));

            a.z = zsign * RAD2DEG * atan2(q.x*q.y + q.w*q.z,
                    0.5f - q.y*q.y - q.z*q.z);
        }

        // https://en.wikipedia.org/wiki/
        //  Conversion_between_quaternions_and_Euler_angles
        static void euler2quat(const axis3_t & a, axis4_t & q)
        {
            // Abbreviations for the various angular functions

            double cr = cos(a.x * 0.5);
            double sr = sin(a.x * 0.5);
            double cp = cos(a.y * 0.5);
            double sp = sin(a.y * 0.5);
            double cy = cos(a.z * 0.5);
            double sy = sin(a.z * 0.5);

            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
        }

        // https://en.wikipedia.org/wiki/Rotation_matrix
        static void angles2rotation(const axis3_t & a, float r[3][3]) 
        {
            const auto alpha = a.z;
            const auto beta = a.y;
            const auto gamma = a.x;

            r[0][0] = cos(beta) * cos(gamma);
            r[0][1] = sin(alpha) * sin(beta) * cos(gamma) -
                cos(alpha) * sin(gamma);
            r[0][2] = cos(alpha) * sin(beta) + sin(alpha) * sin(gamma);
            r[1][0] = cos(beta) * sin(gamma);
            r[1][1] = sin(alpha) * sin(beta) * sin(gamma) +
                cos(alpha) * cos(gamma);
            r[1][2] = cos(alpha) * sin(beta) - sin(alpha) * cos(gamma);
            r[2][0] = -sin(beta);
            r[2][1] = sin(alpha) * cos(beta);
            r[2][2] = cos(alpha) * cos(beta);
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
