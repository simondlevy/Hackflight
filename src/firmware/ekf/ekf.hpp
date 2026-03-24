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

// We want to use F for the Jacobian and _P for the covariance matrix, but
// Arduino pre-defines them
#ifdef F
#undef F
#undef _P
#endif

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

            // Small number epsilon, to prevent dividing by zero
            static constexpr float EPSILON = 1e-6f;

            // the reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            static constexpr float MAX_VELOCITY_MPS = 10;

        public:

            EKF()
            {
                reset(0);
            }

            EKF& operator=(const EKF& other) = default;

            EKF(
                    const ImuFiltered & imuLatest,
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
                    _imuLatest(imuLatest),
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

            auto getVehicleState(
                    const uint32_t msec_curr,
                    const bool isFlying,
                    const uint32_t prediction_freq=100) -> VehicleState
            {
                if (_didResetEstimation) {
                    reset(msec_curr);
                    _didResetEstimation = false;
                }

                // Periodically run the system dynamics to predict the state
                if (Timer::ready(msec_curr, _msec_prev, prediction_freq)) {
                    const auto dt = (msec_curr - _lastPredictionMs) / 1000.0f;
                    _pred = Prediction::run(_pred, dt, isFlying, _R);
                    _isUpdated = true;
                    _lastPredictionMs = msec_curr;
                    _msec_prev = msec_curr;
                }

                const float dt = (msec_curr - _lastProcessNoiseUpdateMs) / 1000.0f;

                if (dt > 0) {

                    const float noise[STATE_DIM] = {
                        PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                        PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                        PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                        PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                        MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                        MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                        MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                    };

                    _pred = Prediction::addCovarianceNoise(_pred, noise);
                    _pred = Prediction::enforceSymmetry(_pred);

                    _lastProcessNoiseUpdateMs = msec_curr;
                }

                // Update with queued measurements and flush the queue

                _gyroLatest = _imuLatest.gyroDps;

                // XXX
                _pred._gyroSubSampler = ImuSubSampler::accumulate(
                        _pred._gyroSubSampler, _gyroLatest);

                _pred._accelSubSampler = ImuSubSampler::accumulate(
                        _pred._accelSubSampler, _imuLatest.accelGs);

                const auto z = _pred._x(0);

                if (_isUpdated) {

                    // Incorporate the attitude error (Kalman filter state) with the attitude
                    const float v0 = _pred._x(4);
                    const float v1 = _pred._x(5);
                    const float v2 = _pred._x(6);

                    // Move attitude error into attitude if any of the angle errors are
                    // large enough
                    if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) >
                                0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 &&
                                    fabsf(v2) < 10)) {

                        const float angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPSILON;
                        const float ca = cos(angle / 2.0f);
                        const float sa = sin(angle / 2.0f);
                        const float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

                        // Rotate the vehicle's attitude by the delta quaternion vector
                        // computed above
                        const float tmpq0 = dq[0] * _pred._q(0) - dq[1] * _pred._q(1) - 
                            dq[2] * _pred._q(2) - dq[3] * _pred._q(3);
                        const float tmpq1 = dq[1] * _pred._q(0) + dq[0] * _pred._q(1) + 
                            dq[3] * _pred._q(2) - dq[2] * _pred._q(3);
                        const float tmpq2 = dq[2] * _pred._q(0) - dq[3] * _pred._q(1) + 
                            dq[0] * _pred._q(2) + dq[1] * _pred._q(3);
                        const float tmpq3 = dq[3] * _pred._q(0) + dq[2] * _pred._q(1) - 
                            dq[1] * _pred._q(2) + dq[0] * _pred._q(3);

                        // normalize and store the result
                        float norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                                tmpq3 * tmpq3) + EPSILON;

                        // XXX
                        _pred._q(0) = tmpq0 / norm;
                        _pred._q(1) = tmpq1 / norm;
                        _pred._q(2) = tmpq2 / norm;
                        _pred._q(3) = tmpq3 / norm;
                    }

                    // Convert the new attitude to a rotation matrix, such that we can
                    // rotate body-frame velocity and acc

                    _R(0,0) = _pred._q(0) * _pred._q(0) + _pred._q(1) *
                        _pred._q(1) - _pred._q(2) * _pred._q(2) - _pred._q(3) *
                        _pred._q(3); _R(0,1) = 2 * _pred._q(1) * _pred._q(2) -
                        2 * _pred._q(0) * _pred._q(3);
                    _R(0,2) = 2 * _pred._q(1) * _pred._q(3) + 2 * _pred._q(0) * _pred._q(2);
                    _R(1,0) = 2 * _pred._q(1) * _pred._q(2) + 2 * _pred._q(0) * _pred._q(3);
                    _R(1,1) = _pred._q(0) * _pred._q(0) - _pred._q(1) *
                        _pred._q(1) + _pred._q(2) * _pred._q(2) - _pred._q(3) *
                        _pred._q(3); _R(1,2) = 2 * _pred._q(2) * _pred._q(3) -
                        2 * _pred._q(0) * _pred._q(1);
                    _R(2,0) = 2 * _pred._q(1) * _pred._q(3) - 2 * _pred._q(0) * _pred._q(2);
                    _R(2,1) = 2 * _pred._q(2) * _pred._q(3) + 2 * _pred._q(0) * _pred._q(1);
                    _R(2,2) = _pred._q(0) * _pred._q(0) - _pred._q(1) *
                        _pred._q(1) - _pred._q(2) * _pred._q(2) + _pred._q(3) *
                        _pred._q(3);

                    _pred = Prediction::enforceSymmetry(_pred);

                    _isUpdated = false;
                }

                const auto dx = 0;//_R(0,0)*_x(1) + _R(0,1)*_x(2) + _R(0,2)*_x(3);
                const auto dy = 0;//_R(1,0)*_x(1) + _R(1,1)*_x(2) + _R(1,2)*_x(3); 
                const auto dz = 0;//_R(2,0)*_x(1) + _R(2,1)*_x(2) + _R(2,2)*_x(3);

                const auto phi = Num::RAD2DEG *
                    atan2f(2*(_pred._q(2)*_pred._q(3)+_pred._q(0)* _pred._q(1))
                            , _pred._q(0)*_pred._q(0) - _pred._q(1)*_pred._q(1)
                            - _pred._q(2)*_pred._q(2) +
                            _pred._q(3)*_pred._q(3));

                const auto dphi = _gyroLatest.x;

                const auto theta = Num::RAD2DEG *
                    asinf(-2*(_pred._q(1)*_pred._q(3) -
                                _pred._q(0)*_pred._q(2)));

                const auto dtheta = _gyroLatest.y;

                const auto psi = Num::RAD2DEG *
                    atan2f(2*(_pred._q(1)*_pred._q(2)+_pred._q(0)*
                                _pred._q(3)), _pred._q(0)*_pred._q(0) +
                            _pred._q(1)*_pred._q(1) - _pred._q(2)*_pred._q(2) -
                            _pred._q(3)*_pred._q(3)); 

                const auto dpsi = _gyroLatest.z;

                if (!velInBounds(dx) || !velInBounds(dy) || !velInBounds(dz)) {
                    _didResetEstimation = true;
                }

                return VehicleState(dx, -dy, z, dz, phi, dphi, theta, dtheta,
                        -psi, -dpsi); // make nose-right positive
            }

            static auto enqueueImu(
                    const EKF & ekf, const ImuFiltered & imudata) -> EKF
            {
                return EKF(
                        imudata,
                        ekf._pred,
                        ekf._didResetEstimation,
                        ekf._msec_prev,
                        ekf._accLatest,
                        ekf._gyroLatest,
                        ekf._predictedNX,
                        ekf._predictedNY,
                        ekf._measuredNX,
                        ekf._measuredNY,
                        ekf._R,
                        ekf._isUpdated,
                        ekf._lastPredictionMs,
                        ekf._lastProcessNoiseUpdateMs);
            }

        private:

            // Indexes to access the vehicle's state, stored as a column vector
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

            typedef enum {
                MEASUREMENT_ACCEL,
                MEASUREMENT_GYRO,
                MEASUREMENT_TOF,
                MEASUREMENT_FLOW,
            } measurementType_e;

            ImuFiltered _imuLatest;

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

            // Instance methods ----------------------------------------------

            void reset(const uint32_t msec_curr)
            {
                _pred = Prediction();
                _isUpdated = false;
                _lastPredictionMs = msec_curr;
                _lastProcessNoiseUpdateMs = msec_curr;
            }

            static bool velInBounds(const float vel)
            {
                return fabs(vel) < MAX_VELOCITY_MPS;
            }

    };
}
