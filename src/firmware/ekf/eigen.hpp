/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <ArduinoEigenDense.h>

#include <firmware/ekf/quaternion.hpp>
#include <firmware/ekf/three_axis_subsampler.hpp>
#include <firmware/flow_filter.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/zranger/filter.hpp>
#include <num.hpp>

// We want to use _P for the covariance matrix, but Arduino pre-defines it
#ifdef _P
#undef _P
#undef _B
#undef _C
#undef F
#endif

namespace hf {

    class EKF { 

        private:

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_Z = 1;
            static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_ROLLPITCH = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

            static constexpr float PROC_NOISE_ACCEL_XY = 0.5f;
            static constexpr float PROC_NOISE_ACCEL_Z = 1.0f;
            static constexpr float PROC_NOISE_VEL = 0;
            static constexpr float PROC_NOISE_POS = 0;
            static constexpr float PROC_NOISE_ATT = 0;
            static constexpr float MEAS_NOISE_GYRO_ROLLPITCH = 0.1f; // radians per second
            static constexpr float MEAS_NOISE_GYRO_YAW = 0.1f;       // radians per second

            static constexpr float GRAVITY = 9.81;

            //We do get the measurements in 10x the motion pixels (experimentally measured)
            static constexpr float FLOW_RESOLUTION = 0.1;

            // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
            static constexpr float MAX_COVARIANCE = 100;
            static constexpr float MIN_COVARIANCE = 1e-6;

            // the reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            static constexpr float MIN_ANGLE = 1e-4;
            static constexpr float MAX_ANGLE = 10;

            // Indexes to acceless the vehicle's state, stored as a column vector
            enum
            {
                STATE_Z,
                STATE_VX,
                STATE_VY,
                STATE_VZ,
                STATE_D0,
                STATE_D1,
                STATE_D2,
                STATE_DIM
            };

            typedef Eigen::MatrixXd Matrix;
            typedef Eigen::VectorXd Vector;

        public:

            EKF& operator=(const EKF& other) = default;

            // From predict()
            EKF(
                    const EKF & other,
                    const float z,
                    const float vx,
                    const float vy,
                    const float vz,
                    const Matrix P,
                    const Quaternion q,
                    const uint32_t msec_curr)
            {
                _P = P;

                _q = q;

                _didPredict = true;

                _lastPredictionMs = msec_curr;

                _x(STATE_Z) = z;
                _x(STATE_VX) = vx;
                _x(STATE_VY) = vy;
                _x(STATE_VZ) = vz;
                _x(STATE_D0) = other._x(STATE_D0);
                _x(STATE_D1) = other._x(STATE_D1);
                _x(STATE_D2) = other._x(STATE_D2);

                _gyroLatest = other._gyroLatest;
                _accelSubSampler = other._accelSubSampler;
                _gyroSubSampler = other._gyroSubSampler;
                _predictedNX = other._predictedNX;
                _predictedNY = other._predictedNY;
                _measuredNX = other._measuredNX;
                _measuredNY = other._measuredNY;
                _r00 = other._r00;
                _r01 = other._r01;
                _r02 = other._r02;
                _r10 = other._r10;
                _r11 = other._r11;
                _r12 = other._r12;
                _r20 = other._r20;
                _r21 = other._r21;
                _r22 = other._r22;
                _didUpdateWithFlowDeck = other._didUpdateWithFlowDeck;
                _zrangerFilterLatest = other._zrangerFilterLatest;
                _opticalFlowFilterLatest = other._opticalFlowFilterLatest;
                _lastProcessNoiseUpdateMs = other._lastProcessNoiseUpdateMs;
            }

            EKF()
            {

                // Initialize the rotation matrix
                _r00 = 1;
                _r01 = 0;
                _r02 = 0;
                _r10 = 0;
                _r11 = 1;
                _r12 = 0;
                _r20 = 0;
                _r21 = 0;
                _r22 = 1;

                // Add in the initial process noise 
                auto noise = xzeros();
                noise << 
                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW;

                _P = ekf_addCovarianceNoise(pzeros(), noise);

                _didPredict = false;
                _didUpdateWithFlowDeck = false;
                _lastPredictionMs = 0;
                _lastProcessNoiseUpdateMs = 0;
            }

            static auto predict(
                    const EKF & ekf,
                    const uint32_t msec_curr,
                    bool isFlying) -> EKF
            {
                const auto accelSubSampler =
                    ThreeAxisSubSampler::finalize(ekf._accelSubSampler);
                const auto gyroSubSampler =
                    ThreeAxisSubSampler::finalize(ekf._gyroSubSampler);

                const auto dt = (msec_curr - ekf._lastPredictionMs) / 1000.f;

                const auto accel = accelSubSampler.subSample;
                const auto gyro = gyroSubSampler.subSample;

                // The linearized Jacobean matrix
                const auto F = makeJacobian(ekf, dt, gyro);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
                const auto P = F * ekf._P * F.transpose();

                // -----------------------------------------------------

                const auto dt2 = dt * dt;

                // keep previous time step's state for the update
                const auto tmpSPX = ekf._x(STATE_VX);
                const auto tmpSPY = ekf._x(STATE_VY);
                const auto tmpSPZ = ekf._x(STATE_VZ);

                // position updates in the body frame (will be rotated to inertial frame)
                const auto dx = ekf._x(STATE_VX) * dt + (isFlying ? 0 : accel.x * dt2 / 2.0f);
                const auto dy = ekf._x(STATE_VY) * dt + (isFlying ? 0 : accel.y * dt2 / 2.0f);

                // thrust can only be produced in the body's Z direction
                const auto dz = ekf._x(STATE_VZ) * dt + accel.z * dt2 / 2.0f; 

                // position update
                const auto z = ekf._x(STATE_Z) +
                    ekf._r20 * dx + ekf._r21 * dy + ekf._r22 * dz -
                    GRAVITY * dt2 / 2.0f;

                const auto accelx = isFlying ? 0.f : accel.x;
                const auto accely = isFlying ? 0.f : accel.y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame

                const auto vx = ekf._x(STATE_VX) +
                    dt * (accelx + gyro.z * tmpSPY -
                            gyro.y * tmpSPZ - GRAVITY * ekf._r20);

                const auto vy = ekf._x(STATE_VY) +
                    dt * (accely - gyro.z * tmpSPX +
                            gyro.x * tmpSPZ - GRAVITY * ekf._r21);

                const auto vz = ekf._x(STATE_VZ) +
                    dt * (accel.z + gyro.y * tmpSPX -
                            gyro.x * tmpSPY - GRAVITY * ekf._r22);


                // Attitude update (rotate by gyroscope): we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const auto dtw = gyro * dt;

                // compute the quaternion values in [w,x,y,z] order
                auto tmpq = rotate(dtw, ekf._q);

                const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                const auto newtmpq = isFlying ? tmpq :
                    Quaternion(
                            keep * tmpq.w + ROLLPITCH_ZERO_REVERSION,
                            keep * tmpq.x,
                            keep * tmpq.y,
                            tmpq.z); 

                // normalize and store the result
                const auto q = newtmpq / Quaternion::l2norm(newtmpq);

                return EKF(ekf, z, vx, vy, vz, P, q, msec_curr);

            } // predict

            static auto update(
                    const EKF & ekf,
                    const ImuFilter::Data & imudata,
                    const uint32_t msec_curr) -> EKF
            {
#if 0
                addProcessNoise(msec_curr);

                _accelSubSampler = ThreeAxisSubSampler::accumulate(
                        _accelSubSampler, imudata.accelGs);

                _gyroSubSampler = ThreeAxisSubSampler::accumulate(
                        _gyroSubSampler, imudata.gyroDps);

                _gyroLatest = imudata.gyroDps;

                if (_didUpdateWithFlowDeck) {
                    updateWithRange(_zrangerFilterLatest);
                    updateWithFlow(_opticalFlowFilterLatest);
                }

                if (_didUpdateWithFlowDeck || _didPredict) {
                    finalize();
                }

                if (_didUpdateWithFlowDeck) {
                    _didUpdateWithFlowDeck = false;
                }

                if (_didPredict) {
                    _didPredict = false;
                }
#endif
                return ekf;
            }

            static auto update(
                    const EKF & ekf,
                    const ZRangerFilter & zrfilter,
                    const OpticalFlowFilter & offilter)
            {
                return ekf;
            }

            static auto getVehicleState(const EKF & ekf) -> VehicleState
            {
                const auto x = ekf._x;

                const auto dx =
                    ekf._r00*x[STATE_VX] +
                    ekf._r01*x[STATE_VY] +
                    ekf._r02*x[STATE_VZ];

                // make right positive
                const auto dy = -(
                        ekf._r10*x[STATE_VX] +
                        ekf._r11*x[STATE_VY] +
                        ekf._r12*x[STATE_VZ]); 

                const auto z = x[STATE_Z];

                const auto dz =
                    ekf._r20*x[STATE_VX] +
                    ekf._r21*x[STATE_VY] +
                    ekf._r22*x[STATE_VZ];

                const auto q0 = ekf._q.w;
                const auto q1 = ekf._q.x;
                const auto q2 = ekf._q.y;
                const auto q3 = ekf._q.z;

                const auto phi = Num::RAD2DEG * atan2f(2*(q2*q3+q0* q1) ,
                        q0*q0 - q1*q1 - q2*q2 + q3*q3);

                const auto dphi = ekf._gyroLatest.x;

                const auto theta = Num::RAD2DEG * asinf(-2*(q1*q3 - q0*q2));

                const auto dtheta = ekf._gyroLatest.y;

                const auto psi = Num::RAD2DEG * atan2f(2*(q1*q2+q0* q3),
                        q0*q0 + q1*q1 - q2*q2 - q3*q3); 

                const auto dpsi = ekf._gyroLatest.z;

                // Return psi/dpsi nose-right positive
                return VehicleState(
                        dx, dy, z, dz, phi, dphi, theta, dtheta, -psi, -dpsi);
            }

        private:

            //////////////////////////////////////////////////////////////////

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Quaternion _q;

            ThreeAxis _gyroLatest;

            ThreeAxisSubSampler _accelSubSampler = ThreeAxisSubSampler(GRAVITY);
            ThreeAxisSubSampler _gyroSubSampler = ThreeAxisSubSampler(Num::DEG2RAD);

            float _predictedNX;
            float _predictedNY;

            float _measuredNX;
            float _measuredNY;

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            float _r00, _r01, _r02, _r10, _r11, _r12, _r20, _r21, _r22; 

            bool _didPredict;

            bool _didUpdateWithFlowDeck;

            ZRangerFilter _zrangerFilterLatest;
            OpticalFlowFilter _opticalFlowFilterLatest;

            uint32_t _lastPredictionMs;
            uint32_t _lastProcessNoiseUpdateMs;

            // State vector
            Vector _x;

            // Covariance matrix
            Matrix _P;

            //////////////////////////////////////////////////////////////////

            static auto makeJacobian(
                    const EKF & ekf,
                    const float dt,
                    const ThreeAxis & gyro) -> Matrix
            {
                const auto vx = ekf._x(STATE_VX);
                const auto vy = ekf._x(STATE_VY);
                const auto vz = ekf._x(STATE_VZ);

                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                auto F = pzeros();

                // position
                F(STATE_Z,STATE_Z) = 1;

                // position from body-frame velocity
                F(STATE_Z,STATE_VX) = ekf._r20*dt;

                F(STATE_Z,STATE_VY) = ekf._r21*dt;

                F(STATE_Z,STATE_VZ) = ekf._r22*dt;

                // position from attitude error
                F(STATE_Z,STATE_D0) = (vy*ekf._r22 - vz*ekf._r21)*dt;

                F(STATE_Z,STATE_D1) = (-vx*ekf._r22 + vz*ekf._r20)*dt;

                F(STATE_Z,STATE_D2) = (vx*ekf._r21 - vy*ekf._r20)*dt;

                // body-frame velocity from body-frame velocity
                F(STATE_VX,STATE_VX) = 1; //drag negligible
                F(STATE_VY,STATE_VX) =-gyro.z*dt;
                F(STATE_VZ,STATE_VX) = gyro.y*dt;

                F(STATE_VX,STATE_VY) = gyro.z*dt;
                F(STATE_VY,STATE_VY) = 1; //drag negligible
                F(STATE_VZ,STATE_VY) =-gyro.x*dt;

                F(STATE_VX,STATE_VZ) =-gyro.y*dt;
                F(STATE_VY,STATE_VZ) = gyro.x*dt;
                F(STATE_VZ,STATE_VZ) = 1; //drag negligible

                // body-frame velocity from attitude error
                F(STATE_VX,STATE_D0) =  0;
                F(STATE_VY,STATE_D0) = -GRAVITY*ekf._r22*dt;
                F(STATE_VZ,STATE_D0) =  GRAVITY*ekf._r21*dt;

                F(STATE_VX,STATE_D1) =  GRAVITY*ekf._r22*dt;
                F(STATE_VY,STATE_D1) =  0;
                F(STATE_VZ,STATE_D1) = -GRAVITY*ekf._r20*dt;

                F(STATE_VX,STATE_D2) = -GRAVITY*ekf._r21*dt;
                F(STATE_VY,STATE_D2) =  GRAVITY*ekf._r20*dt;
                F(STATE_VZ,STATE_D2) =  0;

                F(STATE_D0,STATE_D0) =  1 - d1*d1/2 - d2*d2/2;
                F(STATE_D0,STATE_D1) =  d2 + d0*d1/2;
                F(STATE_D0,STATE_D2) = -d1 + d0*d2/2;

                F(STATE_D1,STATE_D0) = -d2 + d0*d1/2;
                F(STATE_D1,STATE_D1) =  1 - d0*d0/2 - d2*d2/2;
                F(STATE_D1,STATE_D2) =  d0 + d1*d2/2;

                F(STATE_D2,STATE_D0) =  d1 + d0*d2/2;
                F(STATE_D2,STATE_D1) = -d0 + d1*d2/2;
                F(STATE_D2,STATE_D2) = 1 - d0*d0/2 - d1*d1/2;

                return F;
            }

            static auto ekf_addCovarianceNoise(const Matrix & P,
                    const Vector noise) -> Matrix
            {
                return P + identity() * noise;
            }

            static auto identity() -> Matrix
            {
                return Matrix::Identity(STATE_DIM, STATE_DIM); 
            }

            static auto pzeros() -> Matrix
            {
                return Matrix(STATE_DIM, STATE_DIM); 
            }

            static auto xzeros() -> Vector
            {
                return Vector(STATE_DIM); 
            }

            static auto rotate(
                    const ThreeAxis & v, const Quaternion & q)-> Quaternion
            {
                const auto angle = ThreeAxis::l2norm(v);
                const auto ca = cos(angle / 2);
                const auto sa = sin(angle / 2);
                const auto dq = Quaternion(
                        ca, sa*v.x/angle, sa*v.y/angle, sa*v.z/angle);

                return Quaternion(
                        dq.w*q.w - dq.x*q.x - dq.y*q.y - dq.z*q.z,
                        dq.x*q.w + dq.w*q.x + dq.z*q.y - dq.y*q.z,
                        dq.y*q.w - dq.z*q.x + dq.w*q.y + dq.x*q.z,
                        dq.z*q.w + dq.y*q.x - dq.x*q.y + dq.w*q.z);

            }

    };

}
