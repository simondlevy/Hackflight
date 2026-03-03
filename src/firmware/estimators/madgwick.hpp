/*
   Madgwick filter for IMU sensor fusion

   Adapted from https://github.com/nickrehm/dRehmFlight

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

#include <datatypes.h>
#include <num.hpp>

namespace hf {

    class MadgwickFilter {

        private:

            // Filter parameters - tuned for 2kHz loop rate; Do not touch unless
            // you know what you are doing:
            static constexpr float B_MADGWICK = 0.04; // Madgwick filter param
            static constexpr float B_ACCEL = 0.14;    // Accelerometer LPF
            static constexpr float B_GYRO = 0.1;      // Gyro LPF

        public:

            Vec3 angles;

            MadgwickFilter() = default;

            MadgwickFilter
                (const Vec3 & angles,
                 const Vec4 & quat,
                 const Vec3 & gyro,
                 const Vec3 & accel)
                : angles(angles), _quat(quat), _gyro(gyro), _accel(accel) {}

            MadgwickFilter& operator=(const MadgwickFilter& other) = default;

            void initialize()
            {
                _quat.w = 1;
                _quat.x = 0;
                _quat.y = 0;
                _quat.z = 0;

                _accel.x = 0;
                _accel.y = 0;
                _accel.z = 0;

                _gyro.x = 0;
                _gyro.y = 0;
                _gyro.z = 0;
            }

            static auto run(
                    const MadgwickFilter & mf,
                    const float dt,
                    const Vec3 & gyro,
                    const Vec3 & accel) -> MadgwickFilter
            {
                (void)mf;
                (void)dt;
                (void)gyro;
                (void)accel;

                /*
                // LP filter gyro data
                const auto gx = (1 - B_GYRO) * _gyro.x + B_GYRO * gyro.x;
                const auto gy = (1 - B_GYRO) * _gyro.y + B_GYRO * gyro.y;
                const auto gz = (1 - B_GYRO) * _gyro.z + B_GYRO * gyro.z;

                // LP filter accelerometer data
                const auto ax = (1 - B_ACCEL) * _accel.x + B_ACCEL * accel.x;
                const auto ay = (1 - B_ACCEL) * _accel.y + B_ACCEL * accel.y;
                const auto az = (1 - B_ACCEL) * _accel.z + B_ACCEL * accel.z;

                // Convert gyro degrees/sec to radians/sec
                const auto gxr = gx * Num::DEG2RAD;
                const auto gyr = gy * Num::DEG2RAD;
                const auto gzr = gz * Num::DEG2RAD;

                // Compute rate of change of quaternion from gyro
                auto qdot = Vec4(
                        0.5 * (-_quat.x * gxr - _quat.y * gyr - _quat.z * gzr),
                        0.5 * ( _quat.w * gxr + _quat.y * gzr - _quat.z * gyr),
                        0.5 * ( _quat.w * gyr - _quat.x * gzr + _quat.z * gxr),
                        0.5 * ( _quat.w * gzr + _quat.x * gyr - _quat.y * gxr));

                // Compute feedback only if accelerometer measurement valid
                // (avoids NaN in accelerometer normalisation)
                if ((ax != 0) || (ay != 0) || (az != 0)) {

                    computeFeedback({ax, ay, az}, _quat, qdot);
                }

                // Integrate rate of change of quaternion to yield quaternion
                _quat.w = _quat.w + qdot.w * dt;
                _quat.x = _quat.x + qdot.x * dt;
                _quat.y = _quat.y + qdot.y * dt;
                _quat.z = _quat.z + qdot.z * dt;

                // Normalize quaternion
                normalize(_quat);

                angles.x = Num::RAD2DEG * atan2f(_quat.w*_quat.x + _quat.y*_quat.z,
                        0.5 - _quat.x*_quat.x - _quat.y*_quat.y);

                angles.y = Num::RAD2DEG * asinf(2 * (_quat.x*_quat.z - _quat.w*_quat.y));

                // Negate for nose-right positive
                angles.z = -Num::RAD2DEG * atan2f(_quat.x*_quat.y + _quat.w*_quat.z,
                        0.5 - _quat.y*_quat.y - _quat.z*_quat.z);

                // Store previous IMU readings for next time
                _gyro.x = gx;
                _gyro.y = gy;
                _gyro.z = gz;
                _accel.x = ax;
                _accel.y = ay;
                _accel.z = az;
                */

                return MadgwickFilter();
            }

            void run(
                    const float dt,
                    const Vec3 & gyro,
                    const Vec3 & accel,
                    Vec3 & angles)
            {
                // LP filter gyro data
                const auto gx = (1 - B_GYRO) * _gyro.x + B_GYRO * gyro.x;
                const auto gy = (1 - B_GYRO) * _gyro.y + B_GYRO * gyro.y;
                const auto gz = (1 - B_GYRO) * _gyro.z + B_GYRO * gyro.z;

                // LP filter accelerometer data
                const auto ax = (1 - B_ACCEL) * _accel.x + B_ACCEL * accel.x;
                const auto ay = (1 - B_ACCEL) * _accel.y + B_ACCEL * accel.y;
                const auto az = (1 - B_ACCEL) * _accel.z + B_ACCEL * accel.z;

                // Convert gyro degrees/sec to radians/sec
                const auto gxr = gx * Num::DEG2RAD;
                const auto gyr = gy * Num::DEG2RAD;
                const auto gzr = gz * Num::DEG2RAD;

                // Compute rate of change of quaternion from gyro
                auto qdot = Vec4(
                        0.5 * (-_quat.x * gxr - _quat.y * gyr - _quat.z * gzr),
                        0.5 * ( _quat.w * gxr + _quat.y * gzr - _quat.z * gyr),
                        0.5 * ( _quat.w * gyr - _quat.x * gzr + _quat.z * gxr),
                        0.5 * ( _quat.w * gzr + _quat.x * gyr - _quat.y * gxr));

                // Compute feedback only if accelerometer measurement valid
                // (avoids NaN in accelerometer normalisation)
                if ((ax != 0) || (ay != 0) || (az != 0)) {

                    computeFeedback({ax, ay, az}, _quat, qdot);
                }

                // Integrate rate of change of quaternion to yield quaternion
                _quat.w = _quat.w + qdot.w * dt;
                _quat.x = _quat.x + qdot.x * dt;
                _quat.y = _quat.y + qdot.y * dt;
                _quat.z = _quat.z + qdot.z * dt;

                // Normalize quaternion
                normalize(_quat);

                angles.x = Num::RAD2DEG * atan2f(_quat.w*_quat.x + _quat.y*_quat.z,
                        0.5 - _quat.x*_quat.x - _quat.y*_quat.y);

                angles.y = Num::RAD2DEG * asinf(2 * (_quat.x*_quat.z - _quat.w*_quat.y));

                // Negate for nose-right positive
                angles.z = -Num::RAD2DEG * atan2f(_quat.x*_quat.y + _quat.w*_quat.z,
                        0.5 - _quat.y*_quat.y - _quat.z*_quat.z);

                // Store previous IMU readings for next time
                _gyro.x = gx;
                _gyro.y = gy;
                _gyro.z = gz;
                _accel.x = ax;
                _accel.y = ay;
                _accel.z = az;
            }

        private:

            // Current quaternion
            Vec4 _quat;

            // Previous IMU readings for LPF
            Vec3 _gyro;
            Vec3 _accel;

            static void computeFeedback(
                    const Vec3 & accel,
                    const Vec4 & quat,
                    Vec4 & qdot)
            {
                auto an = Vec3(accel.x, accel.y, accel.z);
                normalize(an);

                // Auxiliary variables to avoid repeated arithmetic
                const auto _2q0 = 2 * quat.w;
                const auto _2q1 = 2 * quat.x;
                const auto _2q2 = 2 * quat.y;
                const auto _2q3 = 2 * quat.z;
                const auto _4q0 = 4 * quat.w;
                const auto _4q1 = 4 * quat.x;
                const auto _4q2 = 4 * quat.y;
                const auto _8q1 = 8 * quat.x;
                const auto _8q2 = 8 * quat.y;
                const auto q0q0 = quat.w * quat.w;
                const auto q1q1 = quat.x * quat.x;
                const auto q2q2 = quat.y * quat.y;
                const auto q3q3 = quat.z * quat.z;

                // Gradient-descent algorithm corrective step
                auto s = Vec4(

                        _4q0 * q2q2 + _2q2 * an.x + _4q0 * q1q1 - _2q1 * an.y,

                        _4q1 * q3q3 - _2q3 * an.x +
                        4 * q0q0 * quat.x - _2q0 * an.y - 
                        _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * an.z,

                        4 * q0q0 * quat.y + _2q0 * an.x +
                        _4q2 * q3q3 - _2q3 * an.y - 
                        _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * an.z,

                        4 * q1q1 * quat.z - _2q1 * an.x +
                        4 * q2q2 * quat.z - _2q2 * an.y);

                // Normalize step magnitude
                normalize(s);

                // Apply feedback step
                qdot.w = qdot.w - B_MADGWICK * s.w;
                qdot.x = qdot.x - B_MADGWICK * s.x;
                qdot.y = qdot.y - B_MADGWICK * s.y;
                qdot.z = qdot.z - B_MADGWICK * s.z;
            }

            static float invsqrt(const float x) 
            {
                return 1 / sqrtf(x);
            }

            static void normalize(Vec3 & v)
            {
                const auto rn = invsqrt(v.x*v.x + v.y*v.y + v.z*v.z);

                v.x = v.x * rn;
                v.y = v.y * rn;
                v.z = v.z * rn;
            }

            static void normalize(Vec4 & v)
            {
                const auto rn = invsqrt(v.w*v.w + v.x*v.x + v.y*v.y + v.z*v.z);

                v.w = v.w * rn;
                v.x = v.x * rn;
                v.y = v.y * rn;
                v.z = v.z * rn;
            }
    };

}
