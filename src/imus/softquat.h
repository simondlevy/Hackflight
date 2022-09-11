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

        class GyroReset {

            public: 
                //uint32_t resetTimeEnd;
                bool resetCompleted;
        };

        class Fusion {
            public:
                uint32_t time;
                Quaternion quat;
                Axes rot;
                GyroReset gyroReset;
        };

        class ImuSensor {

            public:

                Axes values;
                Axes adcf;

                uint32_t count;

                void accumulate(float x, float y, float z)
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
                    uint32_t denom = count * Clock::PERIOD();

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
            float xa = fabsf(x);

            float result =
                sqrtf(1.0f - xa) *
                (1.5707288f + xa *
                 (-0.2121144f + xa *
                  (0.0742610f + (-0.0187293f * xa))));

            if (x < 0.0f)
                return M_PI - result;
            else
                return result;
        }

        static float atan2_approx(const float y, const float x)
        {
            float res, absX, absY;
            absX = fabsf(x);
            absY = fabsf(y);
            res  = absX > absY ? absX : absY;
            if (res) res = (absX < absY ? absX : absY) / res;
            else res = 0.0f;
            res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) *
                        res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 *
                            res + atanPolyCoef6) * res + 1.0f);
            if (absY > absX) res = (M_PI / 2.0f) - res;
            if (x < 0) res = M_PI - res;
            if (y < 0) res = -res;
            return res;
        }

        static float invSqrt(const float x)
        {
            return 1.0f / sqrtf(x);
        }

        static float square(const float x)
        {
            return x * x;
        }

        static void quat2euler(const Quaternion & quat, Axes & angles, Axes & rot)
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

            // Results
            angles.x = atan2_approx(r21, r22); 
            angles.y = (0.5f * M_PI) - acos_approx(-r20);
            angles.z = psi + ((psi < 0) ? 2 * M_PI : 0);

            // Additional output
            rot.x = r20;
            rot.y = r21;
            rot.z = r22;
        }

        static auto mahony(
                const float dt,
                const Axes & gyro,
                const Quaternion & q_old) -> Quaternion
        {
            // Convert gyro degrees to radians
            float gx = Math::deg2rad(gyro.x);
            float gy = Math::deg2rad(gyro.y);
            float gz = Math::deg2rad(gyro.z);

            // Apply proportional and integral feedback, then integrate rate-of-change
            float gx1 = gx * dt / 2;
            float gy1 = gy * dt / 2;
            float gz1 = gz * dt / 2;

            // Update quaternion
            float qw = q_old.w - q_old.x * gx1 - q_old.y * gy1 - q_old.z * gz1;
            float qx = q_old.x + q_old.w * gx1 + q_old.y * gz1 - q_old.z * gy1;
            float qy = q_old.y + q_old.w * gy1 - q_old.x * gz1 + q_old.z * gx1;
            float qz = q_old.z + q_old.w * gz1 + q_old.x * gy1 - q_old.y * gx1;

            // Normalise quaternion
            float norm = invSqrt(square(qw) + square(qx) + square(qy) + square(qz));

            return Quaternion(qw * norm, qx * norm, qy * norm, qz * norm);
        }

        auto getQuaternion(const bool isArmed, uint32_t time) -> Quaternion
        {
            int32_t deltaT = time - m_fusionPrev.time;

            Axes gyroAvg = m_gyroAccum.getAverage();

            float dt = deltaT * 1e-6;

            if (!isArmed) {
            }

            return mahony(dt, gyroAvg, m_fusionPrev.quat);

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

        virtual void getEulerAngles(
                const bool isArmed,
                const uint32_t time,
                VehicleState * vstate) override
        {
            Quaternion quat = getQuaternion(isArmed, time);
            Axes rot;
            Axes angles;
            quat2euler(quat, angles, rot);

            m_fusionPrev.time = time;
            m_fusionPrev.quat.w = quat.w;
            m_fusionPrev.quat.x = quat.x;
            m_fusionPrev.quat.y = quat.y;
            m_fusionPrev.quat.z = quat.z;
            m_fusionPrev.rot.x = rot.x;
            m_fusionPrev.rot.y = rot.y;
            m_fusionPrev.rot.z = rot.z;

            m_gyroAccum.reset();

            vstate->phi = angles.x;
            vstate->theta = angles.y;
            vstate->psi = angles.z;
        }
};
