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

#include <firmware/datatypes.hpp>
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

            VehicleState state;

            MadgwickFilter() : state(), _quat(1, 0, 0, 0), _imudata() {}

            MadgwickFilter
                (const VehicleState & state,
                 const Vec4 & quat,
                 const ImuFiltered & imudata)
                : state(state), _quat(quat), _imudata(imudata) {}

            MadgwickFilter& operator=(const MadgwickFilter& other) = default;

            static auto run(
                    const MadgwickFilter & mf,
                    const float dt,
                    const ImuFiltered & imudata) -> MadgwickFilter
            {
                // LP filter IMU data
                const auto gyrolpf = lpf(imudata.gyroDps, mf._imudata.gyroDps, B_GYRO);
                const auto accellpf = lpf(imudata.accelGs, mf._imudata.accelGs, B_ACCEL);

                // Convert filtered gyro degrees/sec to radians/sec
                const auto gyror = Vec3(
                        gyrolpf.x *Num::DEG2RAD,
                        gyrolpf.y *Num::DEG2RAD,
                        gyrolpf.z *Num::DEG2RAD);

                // Compute rate of change of quaternion from gyro
                const auto qdot = Vec4(
                        0.5 * (-mf._quat.x * gyror.x - mf._quat.y * gyror.y - mf._quat.z * gyror.z),
                        0.5 * ( mf._quat.w * gyror.x + mf._quat.y * gyror.z - mf._quat.z * gyror.y),
                        0.5 * ( mf._quat.w * gyror.y - mf._quat.x * gyror.z + mf._quat.z * gyror.x),
                        0.5 * ( mf._quat.w * gyror.z + mf._quat.x * gyror.y - mf._quat.y * gyror.x));

                // Compute feedback only if accelerometer measurement valid
                // (avoids NaN in accelerometer normalisation)
                const auto qdotf =
                    accellpf.x != 0 || accellpf.y != 0 || accellpf.z != 0 ? 
                    addFeedback(qdot, mf._quat, accellpf) :
                    qdot;


                // Integrate rate of change of quaternion to yield quaternion,
                // then normalize
                const auto quat = normalize(Vec4(
                            mf._quat.w + qdotf.w * dt,
                            mf._quat.x + qdotf.x * dt,
                            mf._quat.y + qdotf.y * dt,
                            mf._quat.z + qdotf.z * dt));

                // Madgwick filter won't give us these
                const float dx = 0;
                const float dy = 0;
                const float z = 0;
                const float dz = 0;

                const auto phi = Num::RAD2DEG *
                    atan2f(quat.w*quat.x + quat.y*quat.z,
                            0.5 - quat.x*quat.x - quat.y*quat.y);

                const float dphi = 0;

                const auto theta = -Num::RAD2DEG *
                    asinf(2 * (quat.x*quat.z - quat.w*quat.y));

                const float dtheta = 0;

                const auto psi = -Num::RAD2DEG *
                    atan2f(quat.x*quat.y + quat.w*quat.z,
                            0.5 - quat.y*quat.y - quat.z*quat.z);

                const float dpsi = 0;

                const auto newstate =
                    VehicleState(
                            dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi);

                return MadgwickFilter(newstate, quat, ImuFiltered(gyrolpf, accellpf));
            }

        private:

            // Current quaternion
            Vec4 _quat;

            // Previous IMU readings for LPF
            ImuFiltered _imudata;

            static auto lpf(
                    const Vec3 & curr,
                    const Vec3 & prev,
                    const float coeff) -> Vec3
            {
                return Vec3(
                        lpf(curr.x, prev.x, coeff),
                        lpf(curr.y, prev.y, coeff),
                        lpf(curr.z, prev.z, coeff));
            }

            static auto lpf(
                    const float curr,
                    const float prev,
                    const float coeff) -> float
            {
                return (1 - coeff) * prev + coeff * curr;
            }

            static auto addFeedback(
                    const Vec4 & qdot,
                    const Vec4 & quat,
                    const Vec3 & accel) -> Vec4
            {

                const auto an = normalize(accel);

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
                const auto s = Vec4(

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
                const auto sn = normalize(s);

                // Apply feedback step
                return Vec4(
                        qdot.w - B_MADGWICK * sn.w,
                        qdot.x - B_MADGWICK * sn.x,
                        qdot.y - B_MADGWICK * sn.y,
                        qdot.z - B_MADGWICK * sn.z);

            }

            static auto invsqrt(const float x) -> float
            {
                return 1 / sqrtf(x);
            }

            static auto normalize(const Vec3 & v) -> Vec3
            {
                const auto rn = invsqrt(v.x*v.x + v.y*v.y + v.z*v.z);

                return Vec3(v.x * rn, v.y * rn, v.z * rn);
            }

            static auto normalize(const Vec4 & v) -> Vec4
            {
                const auto rn = invsqrt(v.w*v.w + v.x*v.x + v.y*v.y + v.z*v.z);

                return Vec4(v.w * rn, v.x * rn, v.y * rn, v.z * rn);
            }
    };

}
