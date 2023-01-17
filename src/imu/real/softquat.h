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

#include <SPI.h>

#include <string.h>

#include "core/axes.h"
#include "core/clock.h"
#include "core/filters/pt2.h"
#include "core/vstate.h"
#include "imu/real.h"

class SoftQuatImu : public RealImu {

    friend class AccelerometerTask;

    private:

        // Constants for trig functions
        static constexpr float atanPolyCoef1  = 3.14551665884836e-07f;
        static constexpr float atanPolyCoef2  = 0.99997356613987f;
        static constexpr float atanPolyCoef3  = 0.14744007058297684f;
        static constexpr float atanPolyCoef4  = 0.3099814292351353f;
        static constexpr float atanPolyCoef5  = 0.05030176425872175f;
        static constexpr float atanPolyCoef6  = 0.1471039133652469f;
        static constexpr float atanPolyCoef7  = 0.6444640676891548f;

        // Acceleromter params
        static const uint32_t  ACCEL_SAMPLE_RATE     = 1000;
        static constexpr float ACCEL_LPF_CUTOFF_FREQ = 10;

        // Any interrupt interval less than this will be recognised as the
        // short interval of ~79us
        static const uint8_t SHORT_THRESHOLD = 82 ;

        class Quaternion {

            public:

                float w;
                float x;
                float y;
                float z;

                Quaternion(const float _w, const float _x, const float _y, const float _z)
                {
                    w = _w;
                    x = _x;
                    y = _y;
                    z = _z;
                }

                Quaternion(void)
                    : Quaternion(0, 0, 0, 0)
                {
                }
        };

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

        Pt2Filter m_accelFilterX = accelFilterInit();
        Pt2Filter m_accelFilterY = accelFilterInit();
        Pt2Filter m_accelFilterZ = accelFilterInit();

        static Pt2Filter accelFilterInit(void)
        {
            return Pt2Filter(ACCEL_LPF_CUTOFF_FREQ, 1. / ACCEL_SAMPLE_RATE);
        }

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

        static auto mahony(
                const float dt,
                const Axes & gyro,
                const Axes & accel,
                const Quaternion & q_old,
                const float Kp = 30.0,
                const float Ki = 0.0) -> Quaternion
        {
            auto gx = deg2rad(gyro.x);
            auto gy = deg2rad(gyro.y);
            auto gz = deg2rad(gyro.z);

            auto ax = accel.x;
            auto ay = accel.y;
            auto az = accel.z;

            auto qw = q_old.w;
            auto qx = q_old.x;
            auto qy = q_old.y;
            auto qz = q_old.z;

            const auto recipAccNorm = square(ax) + square(ay) + square(az);

            if (recipAccNorm > 0.0) {

                static float ix;
                static float iy;
                static float iz;

                // Normalise accelerometer (assumed to measure the direction of
                // gravity in body frame)
                auto recipNorm = 1.0 / sqrt(recipAccNorm);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Estimated direction of gravity in the body frame (factor of
                // two divided out)
                const auto vx = qx * qz - qw * qy;  //to normalize these terms,
                const auto vy = qw * qx + qy * qz;
                const auto vz = qw * qw - 0.5 + qz * qz;

                // Error is cross product between estimated and measured
                // direction of gravity in body frame (half the actual
                // magnitude)
                const auto ex = (ay * vz - az * vy);
                const auto ey = (az * vx - ax * vz);
                const auto ez = (ax * vy - ay * vx);

                // Compute and apply to gyro term the integral feedback, if enabled
                if (Ki > 0.0) {
                    ix += Ki * ex * dt;  // integral error scaled by Ki
                    iy += Ki * ey * dt;
                    iz += Ki * ez * dt;
                    gx += ix;  // apply integral feedback
                    gy += iy;
                    gz += iz;
                }

                // Apply proportional feedback to gyro term
                gx += Kp * ex;
                gy += Kp * ey;
                gz += Kp * ez;
            }

            // Integrate rate of change of quaternion, q cross gyro term

            const auto dtnew = 0.5 * dt;

            gx *= dtnew;   
            gy *= dtnew;
            gz *= dtnew;

            const auto qa = qw;
            const auto qb = qx;
            const auto qc = qy;

            qw += (-qb * gx - qc * gy - qz * gz);
            qx += (qa * gx + qc * gz - qz * gy);
            qy += (qa * gy - qb * gz + qz * gx);
            qz += (qa * gz + qb * gy - qc * gx);

            // renormalise quaternion
            auto recipNorm = 1.0 /
                (sqrt(square(qw) + square(qx) + square(qy) + square(qz)));

            return Quaternion(
                    qw * recipNorm,
                    qx * recipNorm,
                    qy * recipNorm,
                    qz * recipNorm);

        } // mahony

        ImuSensor m_gyroAccum;

        float m_accelScale;

        Axes m_accelAxes;

        auto readAndFilterAccelAxis(Pt2Filter & lpf, const uint8_t k) -> float
        {
            return lpf.apply((float)readRawAccel(k));
        }

        int32_t m_shortPeriod;

    protected:

        SoftQuatImu(
                const rotateFun_t rotateFun,
                const uint16_t gyroScale,
                const uint16_t accelScale)
            : RealImu(rotateFun, gyroScale)
        {
            // Initialize quaternion in upright position
            m_fusionPrev.quat.w = 1;

            m_accelScale = accelScale / 32768.;
        }

        void begin(uint32_t clockSpeed)
        {
            m_shortPeriod = clockSpeed / 1000000 * SHORT_THRESHOLD;

            RealImu::begin(clockSpeed);
        }

        auto readGyroDps(void) -> Axes
        {
            m_gyroAccum.accumulate(
                    m_gyroX.dpsFiltered, m_gyroY.dpsFiltered, m_gyroZ.dpsFiltered);

            return RealImu::readGyroDps();
        }

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            auto quat = mahony(
                    (time - m_fusionPrev.time) * 1e-6,
                    m_gyroAccum.getAverage(),
                    m_accelAxes,
                    m_fusionPrev.quat);

            m_fusionPrev.time = time;
            m_fusionPrev.quat = quat;

            m_gyroAccum.reset();

            return quat2euler(quat.w, quat.x, quat.y, quat.z);
        }

        virtual int16_t readRawAccel(uint8_t k) = 0;

        virtual void updateAccelerometer(void) override
        {
            Axes adc = Axes(
                    readAndFilterAccelAxis(m_accelFilterX, 0),
                    readAndFilterAccelAxis(m_accelFilterY, 1),
                    readAndFilterAccelAxis(m_accelFilterZ, 2));

            m_accelAxes = m_rotateFun(adc);

            // XXX should calibrate too

        }

        uint16_t calculateSpiDivisor(const uint32_t clockSpeed, const uint32_t freq)
        {
            uint32_t clk = clockSpeed / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }


    public:

        void handleInterrupt(uint32_t cycleCounter)
        {
            static uint32_t prevTime;

            // Ideally we'd use a time to capture such information, but
            // unfortunately the port used for EXTI interrupt does not have an
            // associated timer
            uint32_t nowCycles = cycleCounter;
            int32_t gyroLastPeriod = intcmp(nowCycles, prevTime);

            // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
            if ((m_shortPeriod == 0) || (gyroLastPeriod < m_shortPeriod)) {

                m_gyroSyncTime = prevTime;
            }

            prevTime = nowCycles;

            RealImu::handleInterrupt();
        }
};
