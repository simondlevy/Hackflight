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

            //We do get the measurements in 10x the motion pixels (experimentally measured)
            static constexpr float FLOW_RESOLUTION = 0.1;

        public:

            EKF() = default;

            EKF& operator=(const EKF& other) = default;

            EKF(
                    const Prediction & pred,
                    const bool didResetEstimation,
                    const uint32_t msec_prev,
                    const Vec3 & accLatest,
                    const Vec3 & gyroLatest,
                    const float predictedNX,
                    const float predictedNY,
                    const float measuredNX,
                    const float measuredNY,
                    const Eigen::MatrixXd & R,
                    const bool isUpdated,
                    const uint32_t lastPredictionMs,
                    const uint32_t lastProcessNoiseUpdateMs
               )
                :
                    _pred(pred),
                    _didResetEstimation(didResetEstimation),
                    _msec_prev(msec_prev),
                    _accLatest(accLatest),
                    _gyroLatest(gyroLatest),
                    _predictedNX(predictedNX),
                    _predictedNY(predictedNY),
                    _measuredNX(measuredNX),
                    _measuredNY(measuredNY),
                    _R(R),
                    _isUpdated(isUpdated),
                    _lastPredictionMs(lastPredictionMs),
                    _lastProcessNoiseUpdateMs(lastProcessNoiseUpdateMs) { }

            auto run(
                    const uint32_t msec_curr,
                    const ImuFiltered & imudata,
                    const bool isFlying,
                    const uint32_t prediction_freq=100) -> VehicleState
            {
                if (_didResetEstimation) {
                    _pred = Prediction();
                    _isUpdated = false;
                    _lastPredictionMs = msec_curr;
                    _lastProcessNoiseUpdateMs = msec_curr;
                }

                _didResetEstimation = false;

                // Periodically run the system dynamics to predict the state
                if (Timer::ready(msec_curr, _msec_prev, prediction_freq)) {
                    const auto dt = (msec_curr - _lastPredictionMs) / 1000.0f;
                    _pred = Prediction::run(_pred, dt, isFlying, _R);
                    _isUpdated = true;
                    _lastPredictionMs = msec_curr;
                    _msec_prev = msec_curr;
                }

                const float dt = (msec_curr - _lastProcessNoiseUpdateMs) / 1000.0f;

                const float noise[Prediction::STATE_DIM] = {
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                if (dt > 0) {

                    _pred = Prediction::addCovarianceNoise(_pred, noise);
                    _pred = Prediction::enforceSymmetry(_pred);

                    _lastProcessNoiseUpdateMs = msec_curr;
                }

                // Update with queued measurements and flush the queue

                _gyroLatest = imudata.gyroDps;

                _pred = Prediction::accumulateImu(_pred, imudata);

                const auto z = _pred.x(0);

                if (_isUpdated) {

                    _pred = Prediction::tryToToIncorporateAttitude(_pred);

                    // Convert the new attitude to a rotation matrix, such that we can
                    // rotate body-frame velocity and acc
                    _R = quat2rotation(_pred.q);

                    _pred = Prediction::enforceSymmetry(_pred);
                }

                _isUpdated = false;

                const auto dx = 0;//_R(0,0)*_x(1) + _R(0,1)*_x(2) + _R(0,2)*_x(3);
                const auto dy = 0;//_R(1,0)*_x(1) + _R(1,1)*_x(2) + _R(1,2)*_x(3); 
                const auto dz = 0;//_R(2,0)*_x(1) + _R(2,1)*_x(2) + _R(2,2)*_x(3);

                const auto q = _pred.q;

                const auto phi = Num::RAD2DEG *
                    atan2f(2*(q(2)*q(3)+q(0)* q(1)),
                            q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3));

                const auto dphi = _gyroLatest.x;

                const auto theta = Num::RAD2DEG *
                    asinf(-2*(q(1)*q(3) - q(0)*q(2)));

                const auto dtheta = _gyroLatest.y;

                const auto psi = Num::RAD2DEG *
                    atan2f(2*(q(1)*q(2)+q(0)* q(3)),
                            q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3)); 

                const auto dpsi = _gyroLatest.z;

                _didResetEstimation =
                    (!Prediction::isVelInBounds(dx) ||
                     !Prediction::isVelInBounds(dy) ||
                     !Prediction::isVelInBounds(dz)) ? true :
                    _didResetEstimation;

                return VehicleState(dx, -dy, z, dz, phi, dphi, theta, dtheta,
                        -psi, -dpsi); // make nose-right positive
            }

       private:

            Prediction _pred;

            bool _didResetEstimation;

            uint32_t _msec_prev;

            Vec3 _accLatest;
            Vec3 _gyroLatest;

            float _predictedNX;
            float _predictedNY;

            float _measuredNX;
            float _measuredNY;

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Eigen::MatrixXd _R = Eigen::MatrixXd(3, 3);

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            bool _isUpdated;

            uint32_t _lastPredictionMs;
            uint32_t _lastProcessNoiseUpdateMs;

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

     };
}
