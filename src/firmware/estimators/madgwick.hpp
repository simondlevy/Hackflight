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

class MadgwickFilter {

    public:

        void initialize()
        {
            _q0 = 1;
            _q1 = 0;
            _q2 = 0;
            _q3 = 0;
        }

        void getEulerAngles(
                const float dt, const axis3_t & gyro, const axis3_t & accel,
                float & phi, float & theta, float & psi)
        {
            // LP filter gyro data
            auto gx = (1 - B_GYRO) * _gx_prev + B_GYRO * gyro.x;
            auto gy = (1 - B_GYRO) * _gy_prev + B_GYRO * gyro.y;
            auto gz = (1 - B_GYRO) * _gz_prev + B_GYRO * gyro.z;
            _gx_prev = gx;
            _gy_prev = gy;
            _gz_prev = gz;

            // LP filter accelerometer data
            auto ax = (1 - B_ACCEL) * _ax_prev + B_ACCEL * accel.x;
            auto ay = (1 - B_ACCEL) * _ay_prev + B_ACCEL * accel.y;
            auto az = (1 - B_ACCEL) * _az_prev + B_ACCEL * accel.z;
            _ax_prev = ax;
            _ay_prev = ay;
            _az_prev = az;

            // Convert gyroscope degrees/sec to radians/sec
            gx /= Num::RAD2DEG;
            gy /= Num::RAD2DEG;
            gz /= Num::RAD2DEG;

            // Compute rate of change of quaternion from gyroscope
            auto qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
            auto qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
            auto qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
            auto qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

            // Compute feedback only if accelerometer measurement valid
            // (avoids NaN in accelerometer normalisation)
            if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Auxiliary variables to avoid repeated arithmetic
                const auto _2q0 = 2.0f * _q0;
                const auto _2q1 = 2.0f * _q1;
                const auto _2q2 = 2.0f * _q2;
                const auto _2q3 = 2.0f * _q3;
                const auto _4q0 = 4.0f * _q0;
                const auto _4q1 = 4.0f * _q1;
                const auto _4q2 = 4.0f * _q2;
                const auto _8q1 = 8.0f * _q1;
                const auto _8q2 = 8.0f * _q2;
                const auto q0q0 = _q0 * _q0;
                const auto q1q1 = _q1 * _q1;
                const auto q2q2 = _q2 * _q2;
                const auto q3q3 = _q3 * _q3;

                // Gradient decent algorithm corrective step

                auto s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;

                auto s1 = _4q1 * q3q3 - _2q3 * ax +
                    4.0f * q0q0 * _q1 - _2q0 * ay - 
                    _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;

                auto s2 = 4.0f * q0q0 * _q2 + _2q0 * ax +
                    _4q2 * q3q3 - _2q3 * ay - 
                    _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;

                auto s3 = 4.0f * q1q1 * _q3 - _2q1 * ax +
                    4.0f * q2q2 * _q3 - _2q2 * ay;

                // Normalize step magnitude
                recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 

                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                // Apply feedback step
                qDot1 -= B_MADGWICK * s0;
                qDot2 -= B_MADGWICK * s1;
                qDot3 -= B_MADGWICK * s2;
                qDot4 -= B_MADGWICK * s3;
            }

            // Integrate rate of change of quaternion to yield quaternion
            _q0 += qDot1 * dt;
            _q1 += qDot2 * dt;
            _q2 += qDot3 * dt;
            _q3 += qDot4 * dt;

            // Normalise quaternion
            const auto recipNorm = invSqrt(
                    _q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);

            _q0 *= recipNorm;
            _q1 *= recipNorm;
            _q2 *= recipNorm;
            _q3 *= recipNorm;

            phi = Num::RAD2DEG * atan2f(_q0*_q1 + _q2*_q3,
                    0.5f - _q1*_q1 - _q2*_q2);

            theta = Num::RAD2DEG * asinf(2 * (_q1*_q3 - _q0*_q2));

            psi = Num::RAD2DEG * atan2f(_q1*_q2 + _q0*_q3,
                    0.5f - _q2*_q2 - _q3*_q3);
        }

    private:

        // Filter parameters - tuned for 2kHz loop rate; Do not touch unless
        // you know what you are doing:
        static constexpr float B_MADGWICK = 0.04; // Madgwick filter param
        static constexpr float B_ACCEL = 0.14;    // Accelerometer LPF
        static constexpr float B_GYRO = 0.1;      // Gyro LPF

        // Initialize quaternion for madgwick filter
        float _q0; 
        float _q1;
        float _q2;
        float _q3;

        // Previous IMU values for LPF
        float _ax_prev, _ay_prev, _az_prev;
        float _gx_prev, _gy_prev, _gz_prev;

        static float invSqrt(float x) 
        {
            return 1.0/sqrtf(x);
        }
};
