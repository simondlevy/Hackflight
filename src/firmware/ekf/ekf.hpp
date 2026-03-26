/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2026 Simon D. Levy
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

#include <firmware/datatypes.hpp>
#include <firmware/ekf/imu_subsampler.hpp>
#include <firmware/ekf/prediction.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

namespace hf {

    class EKF { 

        private:

            static constexpr float PROC_NOISE_ACCEL_XY = 0.5f;
            static constexpr float PROC_NOISE_ACCEL_Z = 1.0f;
            static constexpr float PROC_NOISE_VEL = 0;
            static constexpr float PROC_NOISE_POS = 0;
            static constexpr float PROC_NOISE_ATT = 0;
            static constexpr float MEAS_NOISE_GYRO_ROLLPITCH = 0.1f; // radians per second
            static constexpr float MEAS_NOISE_GYRO_YAW = 0.1f;       // radians per second

            static constexpr float G = 9.81;

            // Small number epsilon, to prevent dividing by zero
            static constexpr float EPSILON = 1e-6f;

            // The reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_XY = 100;
            static constexpr float STDEV_INITIAL_POSITION_Z = 1;
            static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_ROLLPITCH = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

            //We do get the measurements in 10x the motion pixels (experimentally measured)
            static constexpr float FLOW_RESOLUTION = 0.1;

            // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
            static constexpr float MAX_COVARIANCE = 100;
            static constexpr float MIN_COVARIANCE = 1e-6;

            static constexpr float MIN_VELOCITY_MPS = 1e-4;
            static constexpr float MAX_VELOCITY_MPS = 10;

            // Indices to access the vehicle's state, stored as a column vector
            enum {
                STATE_Z,
                STATE_VX,
                STATE_VY,
                STATE_VZ,
                STATE_D0,
                STATE_D1,
                STATE_D2,
                STATE_DIM
            };

        public:

            EKF& operator=(const EKF& other) = default;

            EKF() = default;

            void predict(const uint32_t msec_curr, const bool isFlying)
            {
                _pred = Prediction::run(
                        _pred, lag2dt(msec_curr, _lastPredictionMs),
                        isFlying,
                        _R);
                _isUpdated = true;
                _lastPredictionMs = msec_curr;
                _msec_prev = msec_curr;
            }

            auto update(const uint32_t msec_curr,
                    const ImuFiltered & imudata) -> VehicleState
            {
                _pred = _didResetEstimation ? Prediction() : _pred;

                _isUpdated = _didResetEstimation ? false : _isUpdated;

                _lastPredictionMs = _didResetEstimation ? msec_curr :
                    _lastPredictionMs;

                _lastProcessNoiseUpdateMs = _didResetEstimation ? msec_curr :
                    _lastProcessNoiseUpdateMs;

                _didResetEstimation = false;

                const float dt = (msec_curr - _lastProcessNoiseUpdateMs) / 1000.0f;

                const float noise[STATE_DIM] = {
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                _pred.x = dt > 0 ? enforceSymmetry(_pred.x) : _pred.x;

                _pred._P = dt > 0 ? addCovarianceNoise(_pred._P, noise) : _pred._P;

                _pred._P = dt > 0 ? enforceSymmetry(_pred._P) : _pred._P;

                _lastProcessNoiseUpdateMs = dt > 0 ? msec_curr :
                    _lastProcessNoiseUpdateMs;

                const auto gyroLatest = imudata.gyroDps;

                _pred._gyroSubSampler =
                    ImuSubSampler::accumulate(_pred._gyroSubSampler,
                            imudata.gyroDps);

                _pred._accelSubSampler =
                    ImuSubSampler::accumulate(_pred._accelSubSampler,
                            imudata.accelGs);

                const auto z = _pred.x(0);

                _pred.q = _isUpdated ? 
                    tryToToIncorporateAttitude(_pred.q, _pred.x) : _pred.q;

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and acc
                _R = _isUpdated ? quat2rotation(_pred.q) : _R;

                _pred._P = _isUpdated ?
                    enforceSymmetry(addCovarianceNoise(_pred._P, noise)) :
                    _pred._P;

                _pred.x = _isUpdated ? enforceSymmetry(_pred.x) : _pred.x;

                _isUpdated = false;

                const auto dx = 0;//_R(0,0)*_x(1) + _R(0,1)*_x(2) + _R(0,2)*_x(3);
                const auto dy = 0;//_R(1,0)*_x(1) + _R(1,1)*_x(2) + _R(1,2)*_x(3); 
                const auto dz = 0;//_R(2,0)*_x(1) + _R(2,1)*_x(2) + _R(2,2)*_x(3);

                const auto q = _pred.q;

                const auto phi = Num::RAD2DEG *
                    atan2f(2*(q(2)*q(3)+q(0)* q(1)),
                            q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3));

                const auto dphi = gyroLatest.x;

                const auto theta = Num::RAD2DEG *
                    asinf(-2*(q(1)*q(3) - q(0)*q(2)));

                const auto dtheta = gyroLatest.y;

                const auto psi = Num::RAD2DEG *
                    atan2f(2*(q(1)*q(2)+q(0)* q(3)),
                            q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3)); 

                const auto dpsi = gyroLatest.z;

                _didResetEstimation =
                    (!Prediction::isVelInBounds(dx) ||
                     !Prediction::isVelInBounds(dy) ||
                     !Prediction::isVelInBounds(dz)) ? true :
                    _didResetEstimation;

                return VehicleState(dx, -dy, z, dz, phi, dphi, theta, dtheta,
                        -psi, -dpsi); // make nose-right positive
            }

        private:


            bool _didResetEstimation;

            uint32_t _msec_prev;

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Eigen::MatrixXd _R = Eigen::MatrixXd(3, 3);

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            bool _isUpdated;

            uint32_t _lastPredictionMs;
            uint32_t _lastProcessNoiseUpdateMs;

            Prediction _pred;

            static auto initializeGyroSubSampler() -> ImuSubSampler
            {
                return ImuSubSampler(Num::DEG2RAD);
            }

            static auto initializeAccelSubSampler() -> ImuSubSampler
            {
                return ImuSubSampler(G);
            }


            static auto initializeState() -> Eigen::VectorXd
            {
                return Eigen::VectorXd(STATE_DIM);
            }

            static auto initializeQuaternion() -> Eigen::VectorXd
            {
                auto q = Eigen::VectorXd(4);
                q << 1, 0, 0, 0;
                return q;
            }

            static auto initializeCovarianceMatrix() -> Eigen::MatrixXd
            {
                auto P = Eigen::MatrixXd(STATE_DIM, STATE_DIM);

                const float pinit[STATE_DIM] = {
                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                return addCovarianceNoise(P, pinit);
            }

            static auto lag2dt(const uint32_t usec_curr,
                    const uint32_t usec_prev) -> float
            {
                return (usec_curr - usec_prev) / 1000.f;
            }

            static auto addCovarianceNoise(const Eigen::MatrixXd & P,
                    const float * noise) -> Eigen::MatrixXd
            {
                auto Pcov = P;

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    Pcov(k,k) += noise[k] * noise[k];
                }

                return Pcov;
            }

            static auto enforceSymmetry(const Eigen::MatrixXd & P) ->
                Eigen::MatrixXd
                {
                    Eigen::MatrixXd Psym = Eigen::MatrixXd(STATE_DIM, STATE_DIM);

                    for (int i=0; i<STATE_DIM; i++) {

                        for (int j=i; j<STATE_DIM; j++) {

                            const auto pval = (P(i,j) + P(j,i)) / 2;

                            Psym(i,j) = Psym(j,i) = 
                                isnan(pval) || pval > MAX_COVARIANCE ? MAX_COVARIANCE :
                                i==j && pval < MIN_COVARIANCE ? MIN_COVARIANCE :
                                pval;
                        }

                    }

                    return Psym;
                }

            static auto enforceSymmetry(const Eigen::VectorXd & x)
                -> Eigen::VectorXd
                {
                    Eigen::VectorXd xsym = Eigen::VectorXd(STATE_DIM);
                    xsym << x(0), x(1), x(2), x(3), 0, 0, 0;
                    return xsym;

                }

            static auto tryToToIncorporateAttitude(
                    const Eigen::VectorXd & q,
                    const Eigen::VectorXd & x) -> Eigen::VectorXd
            {
                // Incorporate the attitude error (Kalman filter state) with the attitude
                const float v0 = x(4);
                const float v1 = x(5);
                const float v2 = x(6);

                return ((isVelPositive(v0) || isVelPositive(v1) || isVelPositive(v2))
                        && (isVelInBounds(v0) && isVelInBounds(v1) &&
                            isVelInBounds(v2))) ?
                    incorporateAttitude(v0, v1, v2, q) : q;
            }

            static auto incorporateAttitude(
                    const float v0, const float v1, const float v2,
                    const Eigen::VectorXd & q) -> Eigen::VectorXd
            {
                const float angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPSILON;
                const float ca = cos(angle / 2.0f);
                const float sa = sin(angle / 2.0f);
                const float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

                // Rotate the vehicle's attitude by the delta quaternion vector
                // computed above
                const float q0 = dq[0] * q(0) - dq[1] * q(1) - dq[2] * q(2)
                    - dq[3] * q(3);
                const float q1 = dq[1] * q(0) + dq[0] * q(1) + dq[3] * q(2)
                    - dq[2] * q(3);
                const float q2 = dq[2] * q(0) - dq[3] * q(1) + dq[0] * q(2)
                    + dq[1] * q(3);
                const float q3 = dq[3] * q(0) + dq[2] * q(1) - dq[1] * q(2)
                    + dq[0] * q(3);

                // normalize and store the result
                const float norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + 
                        q3 * q3) + EPSILON;

                Eigen::VectorXd qnew = Eigen::VectorXd(4);
                qnew << q0/norm, q1/norm, q2/norm, q3/norm;

                return qnew;
            }

            static auto makeJacobian(
                    const Eigen::VectorXd & x,
                    const Eigen::MatrixXd & R,
                    const Vec3 & gyro,
                    const Vec3 & accel,
                    const float dt) -> Eigen::MatrixXd
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x(1);
                const auto vy = x(2);
                const auto vz = x(3);

                Eigen::MatrixXd F(STATE_DIM, STATE_DIM);

                // position
                F(0, 0) = 1;

                // position from body-frame velocity
                F(0,1) = R(2,0)*dt;

                F(0,2) = R(2,1)*dt;

                F(0,3) = R(2,2)*dt;

                // position from attitude error
                F(0,4) = (vy*R(2,2) - vz*R(2,1))*dt;

                F(0,5) = (-vx*R(2,2) + vz*R(2,0))*dt;

                F(0,6) = (vx*R(2,1) - vy*R(2,0))*dt;

                // body-frame velocity from body-frame velocity
                F(1,1) = 1; //drag negligible
                F(2,1) =-gyro.z*dt;
                F(3,1) = gyro.y*dt;

                F(1,2) = gyro.z*dt;
                F(2,2) = 1; //drag negligible
                F(3,2) =-gyro.x*dt;

                F(1,3) =-gyro.y*dt;
                F(2,3) = gyro.x*dt;
                F(3,3) = 1; //drag negligible

                // body-frame velocity from attitude error
                F(1,4) =  0;
                F(2,4) = -G*R(2,2)*dt;
                F(3,4) =  G*R(2,1)*dt;

                F(1,5) =  G*R(2,2)*dt;
                F(2,5) =  0;
                F(3,5) = -G*R(2,0)*dt;

                F(1,6) = -G*R(2,1)*dt;
                F(2,6) =  G*R(2,0)*dt;
                F(3,6) =  0;

                F(4,4) =  1 - d1*d1/2 - d2*d2/2;
                F(4,5) =  d2 + d0*d1/2;
                F(4,6) = -d1 + d0*d2/2;

                F(5,4) = -d2 + d0*d1/2;
                F(5,5) =  1 - d0*d0/2 - d2*d2/2;
                F(5,6) =  d0 + d1*d2/2;

                F(6,4) =  d1 + d0*d2/2;
                F(6,5) = -d0 + d1*d2/2;
                F(6,6) = 1 - d0*d0/2 - d1*d1/2;

                return F;
            }


            static auto quat2rotation(
                    const Eigen::VectorXd & q) -> Eigen::MatrixXd
            {
                auto R = Eigen::MatrixXd(3, 3);

                R <<
                    q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
                    2 * q(1) * q(2) - 2 * q(0) * q(3),
                    2 * q(1) * q(3) + 2 * q(0) * q(2),
                    2 * q(1) * q(2) + 2 * q(0) * q(3),
                    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
                    2 * q(2) * q(3) - 2 * q(0) * q(1),
                    2 * q(1) * q(3) - 2 * q(0) * q(2),
                    2 * q(2) * q(3) + 2 * q(0) * q(1),
                    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

                return R;
            }

            static bool isVelInBounds(const float vel)
            {
                return fabs(vel) < MAX_VELOCITY_MPS;
            }

            static bool isVelPositive(const float vel)
            {
                return fabs(vel) > MIN_VELOCITY_MPS;
            }
    };
}
