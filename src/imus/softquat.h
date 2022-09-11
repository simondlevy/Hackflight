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

        typedef struct {
            uint32_t time;
            quaternion_t quat;
            rotation_t rot;
            gyro_reset_t gyroReset;
        } fusion_t;

        fusion_t m_fusionPrev;

        // http://http.developer.nvidia.com/Cg/acos.html Handbook of
        // Mathematical Functions M. Abramowitz and I.A. Stegun, Ed.
        // acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03
        // degree)
        static float acos_approx(float x)
        {
            float xa = fabsf(x);
            float result = sqrtf(1.0f - xa) *
                (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
            if (x < 0.0f)
                return M_PI - result;
            else
                return result;
        }

        static float atan2_approx(float y, float x)
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

        static void quat2euler(
                Imu::quaternion_t * quat, VehicleState * state, Imu::rotation_t * rot)
        {
            float qw = quat->w;
            float qx = quat->x;
            float qy = quat->y;
            float qz = quat->z;

            float r00 = 1 - 2 * qy*qy - 2 * qz*qz;
            float r10 = 2 * (qx*qy + qw*qz);
            float r20 = 2 * (qx*qz - qw*qy);
            float r21 = 2 * (qy*qz + qw*qx);
            float r22 = 1 - 2 * qx*qx - 2 * qy*qy;

            float psi = -atan2_approx(r10, r00); 

            // Results
            state->phi   = atan2_approx(r21, r22); 
            state->theta = (0.5f * M_PI) - acos_approx(-r20);
            state->psi   = psi + ((psi < 0) ? 2 * M_PI : 0);

            // Additional output
            rot->r20 = r20;
            rot->r21 = r21;
            rot->r22 = r22;
        }

        static float invSqrt(float x)
        {
            return 1.0f / sqrtf(x);
        }

        static float square(float x)
        {
            return x * x;
        }

        typedef struct {
            Axes values;
            uint32_t count;
        } imu_sensor_t;

        static void getAverage(imu_sensor_t * sensor, uint32_t period, Axes * avg)
        {
            uint32_t denom = sensor->count * period;

            avg->x = denom ? sensor->values.x / denom : 0;
            avg->y = denom ? sensor->values.y / denom : 0;
            avg->z = denom ? sensor->values.z / denom : 0;
        }

        static void mahony(
                float dt,
                Axes * gyro,
                Imu::quaternion_t * quat_old,
                Imu::quaternion_t * quat_new)
        {
            // Convert gyro degrees to radians
            float gx = Math::deg2rad(gyro->x);
            float gy = Math::deg2rad(gyro->y);
            float gz = Math::deg2rad(gyro->z);

            // Apply proportional and integral feedback, then integrate rate-of-change
            float gx1 = gx * dt / 2;
            float gy1 = gy * dt / 2;
            float gz1 = gz * dt / 2;

            // Update quaternion
            float qw =
                quat_old->w - quat_old->x * gx1 - quat_old->y * gy1 - quat_old->z * gz1;
            float qx =
                quat_old->x + quat_old->w * gx1 + quat_old->y * gz1 - quat_old->z * gy1;
            float qy =
                quat_old->y + quat_old->w * gy1 - quat_old->x * gz1 + quat_old->z * gx1;
            float qz =
                quat_old->z + quat_old->w * gz1 + quat_old->x * gy1 - quat_old->y * gx1;

            // Normalise quaternion
            float recipNorm = invSqrt(square(qw) + square(qx) + square(qy) + square(qz));
            quat_new->w = qw * recipNorm;
            quat_new->x = qx * recipNorm;
            quat_new->y = qy * recipNorm;
            quat_new->z = qz * recipNorm;
        }

        void getQuaternion(bool isArmed, uint32_t time, Imu::quaternion_t * quat)
        {
            int32_t deltaT = time - m_fusionPrev.time;

            Axes gyroAvg = {};
            getAverage(&m_accum, Clock::PERIOD(), &gyroAvg);

            float dt = deltaT * 1e-6;

            Imu::gyro_reset_t new_gyro_reset = {};

            if (!isArmed) {
                memcpy(&m_fusionPrev.gyroReset, &new_gyro_reset,
                        sizeof(Imu::gyro_reset_t));
            }

            mahony(dt, &gyroAvg, &m_fusionPrev.quat, quat);

        }

        imu_sensor_t m_accum;

    public:

        SoftQuatImu(uint16_t gyroScale)
            : Imu(gyroScale)
        {
            memset(&m_fusionPrev, 0, sizeof(fusion_t));

            // Initialize quaternion in upright position
            m_fusionPrev.quat.w = 1;
        }

        virtual void accumulateGyro(float gx, float gy, float gz) override
        {
            static Axes _adcf;

            // integrate using trapezium rule to avoid bias
            m_accum.values.x += 0.5f * (_adcf.x + gx) * Clock::PERIOD();
            m_accum.values.y += 0.5f * (_adcf.y + gy) * Clock::PERIOD();
            m_accum.values.z += 0.5f * (_adcf.z + gz) * Clock::PERIOD();

            m_accum.count++;

            _adcf.x = gx;
            _adcf.y = gy;
            _adcf.z = gz;
        }

        virtual void getEulerAngles(
                const bool isArmed,
                const uint32_t time,
                VehicleState * vstate) override
        {
            Imu::quaternion_t quat = {};
            getQuaternion(isArmed, time, &quat);
            Imu::rotation_t rot = {};
            quat2euler(&quat, vstate, &rot);

            fusion_t fusion;
            fusion.time = time;
            memcpy(&fusion.quat, &quat, sizeof(Imu::quaternion_t));
            memcpy(&fusion.rot, &rot, sizeof(Imu::rotation_t));
            memcpy(&m_fusionPrev, &fusion, sizeof(fusion_t));

            m_accum.count = 0;
            m_accum.values.x = 0;
            m_accum.values.y = 0;
            m_accum.values.z = 0;
         }
};
