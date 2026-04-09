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

#include <firmware/ekf/core.hpp>
#include <firmware/ekf/quaternion.hpp>
#include <firmware/ekf/rotation.hpp>
#include <firmware/ekf/three_axis_subsampler.hpp>
#include <firmware/ekf/tinyekf.hpp>
#include <firmware/flow_filter.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/zranger/filter.hpp>
#include <num.hpp>

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

        public:

            EKF& operator=(const EKF& other) = default;
 
            EKF()
            {
                _core.init();

                // Add in the initial process noise 
                const float pinit[EkfCore::STATE_DIM] = {
                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                _core.addCovarianceNoise(pinit);

                _newcore.addCovarianceNoise(
                        tinyekf::Vector(
                            STDEV_INITIAL_POSITION_Z,
                            STDEV_INITIAL_VELOCITY,
                            STDEV_INITIAL_VELOCITY,
                            STDEV_INITIAL_VELOCITY,
                            STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                            STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                            STDEV_INITIAL_ATTITUDE_YAW));

                _didPredict = false;
                _didUpdateWithFlowDeck = false;
                _lastPredictionMs = 0;
                _lastProcessNoiseUpdateMs = 0;
            }

            void predict(const uint32_t msec_curr, bool isFlying) 
            {
                _accelSubSampler = ThreeAxisSubSampler::finalize(_accelSubSampler);
                _gyroSubSampler = ThreeAxisSubSampler::finalize(_gyroSubSampler);

                const auto dt = (msec_curr - _lastPredictionMs) / 1000.f;

                const auto accel = _accelSubSampler.subSample;
                const auto gyro = _gyroSubSampler.subSample;

                // The linearized Jacobean matrix
                float F[EkfCore::STATE_DIM][EkfCore::STATE_DIM] = {};
                makeJacobian(dt, gyro, _core.x, _R, F);
                _core.predict(F);

                const auto newF = new_makeJacobian(dt, gyro, _core.x, _R);


                const auto dt2 = dt * dt;

                // keep previous time step's state for the update
                const auto tmpSPX = _core.x[1];
                const auto tmpSPY = _core.x[2];
                const auto tmpSPZ = _core.x[3];

                // position updates in the body frame (will be rotated to inertial frame)
                const auto dx = _core.x[1] * dt + (isFlying ? 0 : accel.x * dt2 / 2);
                const auto dy = _core.x[2] * dt + (isFlying ? 0 : accel.y * dt2 / 2);

                // thrust can only be produced in the body's Z direction
                const auto dz = _core.x[3] * dt + accel.z * dt2 / 2; 

                // position update
                _core.x[0] += _R.zx * dx + _R.zy * dy + _R.zz * dz - 
                    GRAVITY * dt2 / 2;

                const auto accelx = isFlying ? 0 : accel.x;
                const auto accely = isFlying ? 0 : accel.y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame

                _core.x[1] += dt * (accelx + gyro.z * tmpSPY - gyro.y * tmpSPZ
                        - GRAVITY * _R.zx);

                _core.x[2] += dt * (accely - gyro.z * tmpSPX + gyro.x * tmpSPZ
                        - GRAVITY * _R.zy);

                _core.x[3] += dt * (accel.z + gyro.y * tmpSPX - gyro.x * tmpSPY
                        - GRAVITY * _R.zz);

                // Attitude update (rotate by gyroscope): we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const auto dtw = gyro * dt;

                // compute the quaternion values in [w,x,y,z] order
                auto tmpq = rotate(dtw, _q);

                if (!isFlying) {

                    const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                    tmpq.w = keep * tmpq.w + ROLLPITCH_ZERO_REVERSION;
                    tmpq.x = keep * tmpq.x; 
                    tmpq.y = keep * tmpq.y; 
                    tmpq.z = keep * tmpq.z; 
                }

                // normalize and store the result
                _q = tmpq / Quaternion::l2norm(tmpq);

                _didPredict = true;
                _lastPredictionMs = msec_curr;

            } // predict

            void update(const ImuFilter::Data & imudata,
                    const uint32_t msec_curr)
            {
                const auto dt =
                    (msec_curr - _lastProcessNoiseUpdateMs) / 1000.0f;

                if (dt > 0) {
                    addProcessNoise(dt, msec_curr);
                    _lastProcessNoiseUpdateMs = msec_curr;
                }

                _accelSubSampler = ThreeAxisSubSampler::accumulate(
                        _accelSubSampler, imudata.accelGs);

                _gyroSubSampler = ThreeAxisSubSampler::accumulate(
                        _gyroSubSampler, imudata.gyroDps);

                _gyroLatest = imudata.gyroDps;

                if (_didUpdateWithFlowDeck) {

                    updateWithRange(_zrangerFilterLatest, _R);

                    updateWithFlow(_opticalFlowFilterLatest, _gyroLatest,_R.zz);
                }

                if (_didUpdateWithFlowDeck || _didPredict) {

                    // Incorporate the attitude error (Kalman filter state) with the attitude
                    const auto v = ThreeAxis(
                            _core.x[4],
                            _core.x[5],
                            _core.x[6]);

                    // Move attitude error into attitude if any of the angle errors are
                    // large enough
                    if ((bigenough(v.x) || bigenough(v.y) || bigenough(v.z)) &&
                            smallenough(v.x) && smallenough(v.y) && smallenough(v.z)) {

                        // normalize and store the result
                        _q = _q / Quaternion::l2norm(rotate(v, _q));
                    }

                    // Convert the new attitude to a rotation matrix, such that we can
                    // rotate body-frame velocity and accel
                    _R = Rotation(
                            _q.w * _q.w + _q.x * _q.x - _q.y * _q.y - _q.z * _q.z,
                            2 * _q.x * _q.y - 2 * _q.w * _q.z,
                            2 * _q.x * _q.z + 2 * _q.w * _q.y,
                            2 * _q.x * _q.y + 2 * _q.w * _q.z,
                            _q.w * _q.w - _q.x * _q.x + _q.y * _q.y - _q.z * _q.z,
                            2 * _q.y * _q.z - 2 * _q.w * _q.x,
                            2 * _q.x * _q.z - 2 * _q.w * _q.y,
                            2 * _q.y * _q.z + 2 * _q.w * _q.x,
                            _q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z);

                    // reset the attitude error
                    _core.x[4] = 0;
                    _core.x[5] = 0;
                    _core.x[6] = 0;

                    enforceSymmetry();
                }

                if (_didUpdateWithFlowDeck) {
                    _didUpdateWithFlowDeck = false;
                }

                if (_didPredict) {
                    _didPredict = false;
                }
            }

            void update(const ZRangerFilter & zrfilter,
                    const OpticalFlowFilter & offilter)
            {
                _zrangerFilterLatest = zrfilter;
                _opticalFlowFilterLatest = offilter;
                _didUpdateWithFlowDeck = true;
            }

            static auto getVehicleState(const EKF & ekf) -> VehicleState
            {
                const auto x = ekf._core.x;

                const auto dx =
                    ekf._R.xx*x[1] +
                    ekf._R.xy*x[2] +
                    ekf._R.xz*x[3];

                // make right positive
                const auto dy = -(
                        ekf._R.yx*x[1] +
                        ekf._R.yy*x[2] +
                        ekf._R.yz*x[3]); 

                const auto z = x[0];

                const auto dz =
                    ekf._R.zx*x[1] +
                    ekf._R.zy*x[2] +
                    ekf._R.zz*x[3];

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

            tinyekf::Core _newcore;

            EkfCore _core;

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Quaternion _q;

            ThreeAxis _gyroLatest;

            ThreeAxisSubSampler _accelSubSampler = ThreeAxisSubSampler(GRAVITY);
            ThreeAxisSubSampler _gyroSubSampler = ThreeAxisSubSampler(Num::DEG2RAD);

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Rotation _R;

            bool _didPredict;

            bool _didUpdateWithFlowDeck;

            ZRangerFilter _zrangerFilterLatest;
            OpticalFlowFilter _opticalFlowFilterLatest;

            uint32_t _lastPredictionMs;
            uint32_t _lastProcessNoiseUpdateMs;

            //////////////////////////////////////////////////////////////////

            static void makeJacobian(
                    const float dt,
                    const ThreeAxis & gyro,
                    const float x[EkfCore::STATE_DIM],
                    const Rotation &R,
                    float F[EkfCore::STATE_DIM][EkfCore::STATE_DIM])
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x[1];
                const auto vy = x[2];
                const auto vz = x[3];

                F[0][0] = 1;
                F[0][1] = R.zx*dt;
                F[0][2] = R.zy*dt;
                F[0][3] = R.zz*dt;
                F[0][4] = (vy*R.zz - vz*R.zy)*dt;
                F[0][5] = (-vx*R.zz + vz*R.zx)*dt;
                F[0][6] = (vx*R.zy - vy*R.zx)*dt;

                F[1][0] = 0;
                F[1][1] = 1;
                F[1][2] = gyro.z*dt;
                F[1][3] =-gyro.y*dt;
                F[1][4] =  0;
                F[1][5] =  GRAVITY*R.zz*dt;
                F[1][6] = -GRAVITY*R.zy*dt;

                F[2][0] = 0;
                F[2][1] =-gyro.z*dt;
                F[2][2] = 1;
                F[2][3] = gyro.x*dt;
                F[2][4] = -GRAVITY*R.zz*dt;
                F[2][5] =  0;
                F[2][6] =  GRAVITY*R.zx*dt;

                F[3][0] = 0;
                F[3][1] = gyro.y*dt;
                F[3][2] =-gyro.x*dt;
                F[3][3] = 1;
                F[3][4] =  GRAVITY*R.zy*dt;
                F[3][5] = -GRAVITY*R.zx*dt;
                F[3][6] =  0;

                F[4][0] = 0;
                F[4][1] = 0;
                F[4][2] = 0;
                F[4][3] = 0;
                F[4][4] = 1 - d1*d1/2 - d2*d2/2;
                F[4][5] =  d2 + d0*d1/2;
                F[4][6] = -d1 + d0*d2/2;

                F[5][0] = 0;
                F[5][1] = 0;
                F[5][2] = 0;
                F[5][3] = 0;
                F[5][4] = -d2 + d0*d1/2;
                F[5][5] =  1 - d0*d0/2 - d2*d2/2;
                F[5][6] =  d0 + d1*d2/2;

                F[6][0] = 0;
                F[6][1] = 0;
                F[6][2] = 0;
                F[6][3] = 0;
                F[6][4] =  d1 + d0*d2/2;
                F[6][5] = -d0 + d1*d2/2;
                F[6][6] = 1 - d0*d0/2 - d1*d1/2;
            }

            static auto new_makeJacobian(
                    const float dt,
                    const ThreeAxis & gyro,
                    const float x[EkfCore::STATE_DIM],
                    const Rotation &R) -> tinyekf::Matrix
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x[1];
                const auto vy = x[2];
                const auto vz = x[3];

                return tinyekf::Matrix();
            }

            void addProcessNoise(const float dt, const uint32_t msec_curr)
            {
                const float noise[EkfCore::STATE_DIM] = {
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                _core.addCovarianceNoise(noise);

                enforceSymmetry();
            }

            void updateWithRange(const ZRangerFilter & zrfilter,
                    const Rotation &R)
            {
                // Updates the filter with a measured distance in the zb direction using the
                float h[EkfCore::STATE_DIM] = {};

                // Only update the filter if the measurement is reliable 
                // (\hat{h} -> infty when R.zz -> 0)
                if (fabs(R.zz) > 0.1f && R.zz > 0) {
                    const auto angle = max(0, fabsf(acosf(R.zz)) -
                            Num::DEG2RAD * (15.0f / 2));
                    const auto predictedDistance = _core.x[0] / cosf(angle);
                    const auto measuredDistance = zrfilter.distance_m;

                    // This just acts like a gain for the sensor model. Further
                    // updates are done in the scalar update function below
                    h[0] = 1 / cosf(angle); 

                    updateWithScalar(h, measuredDistance-predictedDistance,
                            zrfilter.stdev);
                }
            }

            void updateWithFlow(const OpticalFlowFilter & offilter,
                    const ThreeAxis & gyro, const float r22)
            {
                updateWithFlowAxis(offilter.dt, r22, offilter.dpixelx,
                        offilter.stdDevX, 1, gyro.y);

                updateWithFlowAxis(offilter.dt, r22, offilter.dpixely,
                        offilter.stdDevY, 2, gyro.x);
            }

            void updateWithFlowAxis(
                    const float dt,
                    const float r22,
                    const float dpixel,
                    const float stdev,
                    const uint8_t state_index,
                    const float gyroval)
            {
                // [pixels] (same in x and y)
                const float Npix = 35.0;                      

                //float thetapix = Num::DEG2RAD * 4.0f;
                // [rad]    (same in x and y)
                // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
                // corresponding ground length
                const float thetapix = 0.71674f;

                // Saturate elevation in prediction and correction to avoid singularities
                const auto z_g  = max(_core.x[0], 0.1);

                const auto dg = _core.x[state_index];

                const auto omegab = gyroval * Num::DEG2RAD;

                float h[EkfCore::STATE_DIM] = {};

                const auto predictedN = (dt * Npix / thetapix ) * 
                    ((dg * r22 / z_g) - omegab);

                const auto measuredN = dpixel*FLOW_RESOLUTION;

                h[0] = (Npix * dt / thetapix) * ((r22 * dg) / (-z_g * z_g));

                h[state_index] = (Npix * dt / thetapix) * (r22 / z_g);

                // First update
                updateWithScalar(h, measuredN-predictedN,
                        stdev*FLOW_RESOLUTION);
            }

            void enforceSymmetry()
            {
                _core.enforceSymmetry(MIN_COVARIANCE, MAX_COVARIANCE);
            }

            void updateWithScalar(
                    const float * h,
                    const float error,
                    const float stdMeasNoise)
            {
                _core.updateWithScalar(h, error, stdMeasNoise,
                        MIN_COVARIANCE, MAX_COVARIANCE);
            }

            static auto bigenough(const float v) -> bool
            {
                return fabsf(v) > MIN_ANGLE;
            }

            static auto smallenough(const float v) -> bool
            {
                return fabsf(v) < MAX_ANGLE;
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
