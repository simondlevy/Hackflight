/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <string.h>

#include "core/axes.h"
#include "core/clock.h"
#include "core/vstate.h"
#include "imu.h"
#include "maths.h"

class SoftQuatImu : public Imu {

    private:

        // Constants for trig functions

        static constexpr float atanPolyCoef1  = 3.14551665884836e-07f;
        static constexpr float atanPolyCoef2  = 0.99997356613987f;
        static constexpr float atanPolyCoef3  = 0.14744007058297684f;
        static constexpr float atanPolyCoef4  = 0.3099814292351353f;
        static constexpr float atanPolyCoef5  = 0.05030176425872175f;
        static constexpr float atanPolyCoef6  = 0.1471039133652469f;
        static constexpr float atanPolyCoef7  = 0.6444640676891548f;

        class Fusion {
            public:
                uint32_t time;
                Quaternion quat;
                Axes rot;
        };

        class ImuSensor {

            public:

                Axes values;
                Axes adcf;

                uint32_t count;

                void accumulate(const float x, const float y, const float z)
                {
                    // integrate using trapezium rule to avoid bias
                    values.x += 0.5f * (adcf.x + x) * Clock::PERIOD();
                    values.y += 0.5f * (adcf.y + y) * Clock::PERIOD();
                    values.z += 0.5f * (adcf.z + z) * Clock::PERIOD();

                    adcf.x = x;
                    adcf.y = y;
                    adcf.z = z;

                    count++;
                }
                
                Axes getAverage(void)
                {
                    auto denom = count * Clock::PERIOD();

                    return Axes(
                            denom ? values.x / denom : 0,
                            denom ? values.y / denom : 0,
                            denom ? values.z / denom : 0);
                }

                void reset(void)
                {
                    values.x = 0;
                    values.y = 0;
                    values.z = 0;
                    count = 0;
                }
        };

        Fusion m_fusionPrev;

        // http://http.developer.nvidia.com/Cg/acos.html Handbook of
        // Mathematical Functions M. Abramowitz and I.A. Stegun, Ed.
        // acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03
        // degree)
        static float acos_approx(const float x)
        {
            const auto xa = fabsf(x);

            const auto result =
                sqrtf(1.0f - xa) *
                (1.5707288f + xa *
                 (-0.2121144f + xa *
                  (0.0742610f + (-0.0187293f * xa))));

            return x < 0 ? M_PI - result : result;
        }

        static float atan2_approx(const float y, const float x)
        {
            const auto absX = fabsf(x);

            const auto absY = fabsf(y);

            const auto a  = absX > absY ? absX : absY;

            const auto b = a ? (absX < absY ? absX : absY) / a : 0;

            const auto c = -((((atanPolyCoef5 * b - atanPolyCoef4) * b - atanPolyCoef3) *
                        b - atanPolyCoef2) * b - atanPolyCoef1) / ((atanPolyCoef7 *
                            b + atanPolyCoef6) * b + 1.0f);

            const auto d = absY > absX ? (M_PI / 2) - c : c;

            const auto e = x < 0  ? M_PI - d : d;

            return y < 0 ? e : -e;
        }

        static float invSqrt(const float x)
        {
            return 1.0f / sqrtf(x);
        }

        static float square(const float x)
        {
            return x * x;
        }

        static auto quat2euler(const Quaternion & quat, Axes & rot) -> Axes
        {
            const auto qw = quat.w;
            const auto qx = quat.x;
            const auto qy = quat.y;
            const auto qz = quat.z;

            const auto r00 = 1 - 2 * qy*qy - 2 * qz*qz;
            const auto r10 = 2 * (qx*qy + qw*qz);
            const auto r20 = 2 * (qx*qz - qw*qy);
            const auto r21 = 2 * (qy*qz + qw*qx);
            const auto r22 = 1 - 2 * qx*qx - 2 * qy*qy;

            const auto psi = -atan2_approx(r10, r00); 

            // Additional output
            rot.x = r20;
            rot.y = r21;
            rot.z = r22;

            return Axes(
                    atan2_approx(r21, r22),
                    (0.5f * M_PI) - acos_approx(-r20),
                    psi + ((psi < 0) ? 2 * M_PI : 0));
        }

        static auto mahony(
                const float dt,
                const Axes & gyro,
                const Quaternion & q_old) -> Quaternion
        {
            // Convert gyro degrees to radians
            const auto gx = Math::deg2rad(gyro.x);
            const auto gy = Math::deg2rad(gyro.y);
            const auto gz = Math::deg2rad(gyro.z);

            // Apply proportional and integral feedback, then integrate rate-of-change
            const auto gx1 = gx * dt / 2;
            const auto gy1 = gy * dt / 2;
            const auto gz1 = gz * dt / 2;

            // Update quaternion
            const auto qw = q_old.w - q_old.x * gx1 - q_old.y * gy1 - q_old.z * gz1;
            const auto qx = q_old.x + q_old.w * gx1 + q_old.y * gz1 - q_old.z * gy1;
            const auto qy = q_old.y + q_old.w * gy1 - q_old.x * gz1 + q_old.z * gx1;
            const auto qz = q_old.z + q_old.w * gz1 + q_old.x * gy1 - q_old.y * gx1;

            // Normalise quaternion
            float norm = invSqrt(square(qw) + square(qx) + square(qy) + square(qz));

            return Quaternion(qw * norm, qx * norm, qy * norm, qz * norm);
        }

        ImuSensor m_gyroAccum;

    public:

        SoftQuatImu(const uint16_t gyroScale)
            : Imu(gyroScale)
        {
            // Initialize quaternion in upright position
            m_fusionPrev.quat.w = 1;
        }

        virtual void accumulateGyro(
                const float gx, const float gy, const float gz) override
        {
            m_gyroAccum.accumulate(gx, gy, gz);
        }

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            auto quat = mahony(
                    (time - m_fusionPrev.time) * 1e-6,
                    m_gyroAccum.getAverage(),
                    m_fusionPrev.quat);

            m_fusionPrev.time = time;
            m_fusionPrev.quat = quat;

            m_gyroAccum.reset();

            return quat2euler(quat, m_fusionPrev.rot);
        }
};
