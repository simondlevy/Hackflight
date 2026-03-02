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

        public:

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

            void getEulerAngles(
                    const float dt,
                    const Vec3 & gyro,
                    const Vec3 & accel,
                    Vec3 & angles)
            {
                // LP filter gyro data
                auto gx = (1 - B_GYRO) * _gyro.x + B_GYRO * gyro.x;
                auto gy = (1 - B_GYRO) * _gyro.y + B_GYRO * gyro.y;
                auto gz = (1 - B_GYRO) * _gyro.z + B_GYRO * gyro.z;

                _gyro.x = gx;
                _gyro.y = gy;
                _gyro.z = gz;

                // LP filter accelerometer data
                auto ax = (1 - B_ACCEL) * _accel.x + B_ACCEL * accel.x;
                auto ay = (1 - B_ACCEL) * _accel.y + B_ACCEL * accel.y;
                auto az = (1 - B_ACCEL) * _accel.z + B_ACCEL * accel.z;

                _accel.x = ax;
                _accel.y = ay;
                _accel.z = az;

                // Convert gyroscope degrees/sec to radians/sec
                gx /= Num::RAD2DEG;
                gy /= Num::RAD2DEG;
                gz /= Num::RAD2DEG;

                // Compute rate of change of quaternion from gyroscope
                auto qdot = Vec4(
                        0.5 * (-_quat.x * gx - _quat.y * gy - _quat.z * gz),
                        0.5 * (_quat.w * gx + _quat.y * gz - _quat.z * gy),
                        0.5 * (_quat.w * gy - _quat.x * gz + _quat.z * gx),
                        0.5 * (_quat.w * gz + _quat.x * gy - _quat.y * gx));

                // Compute feedback only if accelerometer measurement valid
                // (avoids NaN in accelerometer normalisation)
                if(!((ax == 0) && (ay == 0) && (az == 0))) {

                    // Normalise accelerometer measurement
                    auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                    ax *= recipNorm;
                    ay *= recipNorm;
                    az *= recipNorm;

                    // Auxiliary variables to avoid repeated arithmetic
                    const auto _2q0 = 2 * _quat.w;
                    const auto _2q1 = 2 * _quat.x;
                    const auto _2q2 = 2 * _quat.y;
                    const auto _2q3 = 2 * _quat.z;
                    const auto _4q0 = 4 * _quat.w;
                    const auto _4q1 = 4 * _quat.x;
                    const auto _4q2 = 4 * _quat.y;
                    const auto _8q1 = 8 * _quat.x;
                    const auto _8q2 = 8 * _quat.y;
                    const auto q0q0 = _quat.w * _quat.w;
                    const auto q1q1 = _quat.x * _quat.x;
                    const auto q2q2 = _quat.y * _quat.y;
                    const auto q3q3 = _quat.z * _quat.z;

                    // Gradient decent algorithm corrective step
                    auto s = Vec4(

                            _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay,

                            _4q1 * q3q3 - _2q3 * ax +
                            4 * q0q0 * _quat.x - _2q0 * ay - 
                            _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az,

                            4 * q0q0 * _quat.y + _2q0 * ax +
                            _4q2 * q3q3 - _2q3 * ay - 
                            _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az,

                            4 * q1q1 * _quat.z - _2q1 * ax +
                            4 * q2q2 * _quat.z - _2q2 * ay);

                    // Normalize step magnitude
                    recipNorm = invSqrt(s.w * s.w + s.x * s.x + s.y * s.y + s.z * s.z); 

                    s.w *= recipNorm;
                    s.x *= recipNorm;
                    s.y *= recipNorm;
                    s.z *= recipNorm;

                    // Apply feedback step
                    qdot.w -= B_MADGWICK * s.w;
                    qdot.x -= B_MADGWICK * s.x;
                    qdot.y -= B_MADGWICK * s.y;
                    qdot.z -= B_MADGWICK * s.z;
                }

                // Integrate rate of change of quaternion to yield quaternion
                _quat.w += qdot.w * dt;
                _quat.x += qdot.x * dt;
                _quat.y += qdot.y * dt;
                _quat.z += qdot.z * dt;

                // Normalise quaternion
                const auto recipNorm = invSqrt(
                        _quat.w * _quat.w +
                        _quat.x * _quat.x +
                        _quat.y * _quat.y +
                        _quat.z * _quat.z);

                _quat.w *= recipNorm;
                _quat.x *= recipNorm;
                _quat.y *= recipNorm;
                _quat.z *= recipNorm;

                angles.x = Num::RAD2DEG * atan2f(_quat.w*_quat.x + _quat.y*_quat.z,
                        0.5 - _quat.x*_quat.x - _quat.y*_quat.y);

                angles.y = Num::RAD2DEG * asinf(2 * (_quat.x*_quat.z - _quat.w*_quat.y));

                // Negate for nose-right positive
                angles.z = -Num::RAD2DEG * atan2f(_quat.x*_quat.y + _quat.w*_quat.z,
                        0.5 - _quat.y*_quat.y - _quat.z*_quat.z);
            }

        private:

            // Filter parameters - tuned for 2kHz loop rate; Do not touch unless
            // you know what you are doing:
            static constexpr float B_MADGWICK = 0.04; // Madgwick filter param
            static constexpr float B_ACCEL = 0.14;    // Accelerometer LPF
            static constexpr float B_GYRO = 0.1;      // Gyro LPF

            // Current quaternion
            Vec4 _quat;

            // Previous IMU readings for LPF
            Vec3 _gyro;
            Vec3 _accel;

            static float invSqrt(float x) 
            {
                return 1.0/sqrtf(x);
            }
    };

}
