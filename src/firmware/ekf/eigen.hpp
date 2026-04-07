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

        public:

            EKF& operator=(const EKF& other) = default;
 
            EKF()
            {
                ekf_init(_x, _P);

                // Initialize the rotation matrix
                _R[0][0] = 1;
                _R[0][1] = 0;
                _R[0][2] = 0;
                _R[1][0] = 0;
                _R[1][1] = 1;
                _R[1][2] = 0;
                _R[2][0] = 0;
                _R[2][1] = 0;
                _R[2][2] = 1;

                // Add in the initial process noise 
                const float pinit[STATE_DIM] = {

                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                ekf_addCovarianceNoise(pinit, _P);

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
                float F[STATE_DIM][STATE_DIM] = {};
                makeJacobian(dt, gyro, _x, _R, F);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------

                float FP[STATE_DIM][STATE_DIM] = {};
                dot(F, _P, FP);

                float Ft[STATE_DIM][STATE_DIM] = {};
                trans(F, Ft);

                dot(FP, Ft, _P);

                // -----------------------------------------------------

                const auto dt2 = dt * dt;

                // keep previous time step's state for the update
                const auto tmpSPX = _x[STATE_VX];
                const auto tmpSPY = _x[STATE_VY];
                const auto tmpSPZ = _x[STATE_VZ];

                // position updates in the body frame (will be rotated to inertial frame)
                const auto dx = _x[STATE_VX] * dt + (isFlying ? 0 : accel.x * dt2 / 2);
                const auto dy = _x[STATE_VY] * dt + (isFlying ? 0 : accel.y * dt2 / 2);

                // thrust can only be produced in the body's Z direction
                const auto dz = _x[STATE_VZ] * dt + accel.z * dt2 / 2; 

                // position update
                _x[STATE_Z] += _R[2][0] * dx + _R[2][1] * dy + _R[2][2] * dz - 
                    GRAVITY * dt2 / 2;

                const auto accelx = isFlying ? 0 : accel.x;
                const auto accely = isFlying ? 0 : accel.y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame

                _x[STATE_VX] += dt * (accelx + gyro.z * tmpSPY - gyro.y * tmpSPZ
                        - GRAVITY * _R[2][0]);

                _x[STATE_VY] += dt * (accely - gyro.z * tmpSPX + gyro.x * tmpSPZ
                        - GRAVITY * _R[2][1]);

                _x[STATE_VZ] += dt * (accel.z + gyro.y * tmpSPX - gyro.x * tmpSPY
                        - GRAVITY * _R[2][2]);

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
                    addProcessNoise(dt, msec_curr, _P);
                    _lastProcessNoiseUpdateMs = msec_curr;
                }

                _accelSubSampler = ThreeAxisSubSampler::accumulate(
                        _accelSubSampler, imudata.accelGs);

                _gyroSubSampler = ThreeAxisSubSampler::accumulate(
                        _gyroSubSampler, imudata.gyroDps);

                _gyroLatest = imudata.gyroDps;

                if (_didUpdateWithFlowDeck) {
                    updateWithRange(_zrangerFilterLatest, _R, _x, _P);
                    updateWithFlow(
                            _opticalFlowFilterLatest, _gyroLatest, _R[2][2],
                            _x, _P);
                }

                if (_didUpdateWithFlowDeck || _didPredict) {

                    // Incorporate the attitude error (Kalman filter state) with the attitude
                    const auto v = ThreeAxis(_x[STATE_D0], _x[STATE_D1], _x[STATE_D2]);

                    // Move attitude error into attitude if any of the angle errors are
                    // large enough
                    if ((bigenough(v.x) || bigenough(v.y) || bigenough(v.z)) &&
                            smallenough(v.x) && smallenough(v.y) && smallenough(v.z)) {

                        // normalize and store the result
                        _q = _q / Quaternion::l2norm(rotate(v, _q));
                    }

                    // Convert the new attitude to a rotation matrix, such that we can
                    // rotate body-frame velocity and accel
                    _R[0][0] = _q.w * _q.w + _q.x * _q.x - _q.y * _q.y - _q.z * _q.z;
                    _R[0][1] = 2 * _q.x * _q.y - 2 * _q.w * _q.z;
                    _R[0][2] = 2 * _q.x * _q.z + 2 * _q.w * _q.y;
                    _R[1][0] = 2 * _q.x * _q.y + 2 * _q.w * _q.z;
                    _R[1][1] = _q.w * _q.w - _q.x * _q.x + _q.y * _q.y - _q.z * _q.z;
                    _R[1][2] = 2 * _q.y * _q.z - 2 * _q.w * _q.x;
                    _R[2][0] = 2 * _q.x * _q.z - 2 * _q.w * _q.y;
                    _R[2][1] = 2 * _q.y * _q.z + 2 * _q.w * _q.x;
                    _R[2][2] = _q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z;

                    // reset the attitude error
                    _x[STATE_D0] = 0;
                    _x[STATE_D1] = 0;
                    _x[STATE_D2] = 0;

                    ekf_enforceSymmetry(_P);
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
                const auto x = ekf._x;

                const auto dx =
                    ekf._R[0][0]*x[STATE_VX] +
                    ekf._R[0][1]*x[STATE_VY] +
                    ekf._R[0][2]*x[STATE_VZ];

                // make right positive
                const auto dy = -(
                        ekf._R[1][0]*x[STATE_VX] +
                        ekf._R[1][1]*x[STATE_VY] +
                        ekf._R[1][2]*x[STATE_VZ]); 

                const auto z = x[STATE_Z];

                const auto dz =
                    ekf._R[2][0]*x[STATE_VX] +
                    ekf._R[2][1]*x[STATE_VY] +
                    ekf._R[2][2]*x[STATE_VZ];

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

            // State vector
            __attribute__((aligned(4))) float _x[STATE_DIM];

            // Covariance matrix
            __attribute__((aligned(4))) float _P[STATE_DIM][STATE_DIM];

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Quaternion _q;

            ThreeAxis _gyroLatest;

            ThreeAxisSubSampler _accelSubSampler = ThreeAxisSubSampler(GRAVITY);
            ThreeAxisSubSampler _gyroSubSampler = ThreeAxisSubSampler(Num::DEG2RAD);

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            float _R[3][3];

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
                    const float x[STATE_DIM],
                    const float R[3][3],
                    float F[STATE_DIM][STATE_DIM])
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x[STATE_VX];
                const auto vy = x[STATE_VY];
                const auto vz = x[STATE_VZ];

                // position
                F[STATE_Z][STATE_Z] = 1;

                // position from body-frame velocity
                F[STATE_Z][STATE_VX] = R[2][0]*dt;

                F[STATE_Z][STATE_VY] = R[2][1]*dt;

                F[STATE_Z][STATE_VZ] = R[2][2]*dt;

                // position from attitude error
                F[STATE_Z][STATE_D0] = (vy*R[2][2] - vz*R[2][1])*dt;

                F[STATE_Z][STATE_D1] = (-vx*R[2][2] + vz*R[2][0])*dt;

                F[STATE_Z][STATE_D2] = (vx*R[2][1] - vy*R[2][0])*dt;

                // body-frame velocity from body-frame velocity
                F[STATE_VX][STATE_VX] = 1; //drag negligible
                F[STATE_VY][STATE_VX] =-gyro.z*dt;
                F[STATE_VZ][STATE_VX] = gyro.y*dt;

                F[STATE_VX][STATE_VY] = gyro.z*dt;
                F[STATE_VY][STATE_VY] = 1; //drag negligible
                F[STATE_VZ][STATE_VY] =-gyro.x*dt;

                F[STATE_VX][STATE_VZ] =-gyro.y*dt;
                F[STATE_VY][STATE_VZ] = gyro.x*dt;
                F[STATE_VZ][STATE_VZ] = 1; //drag negligible

                // body-frame velocity from attitude error
                F[STATE_VX][STATE_D0] =  0;
                F[STATE_VY][STATE_D0] = -GRAVITY*R[2][2]*dt;
                F[STATE_VZ][STATE_D0] =  GRAVITY*R[2][1]*dt;

                F[STATE_VX][STATE_D1] =  GRAVITY*R[2][2]*dt;
                F[STATE_VY][STATE_D1] =  0;
                F[STATE_VZ][STATE_D1] = -GRAVITY*R[2][0]*dt;

                F[STATE_VX][STATE_D2] = -GRAVITY*R[2][1]*dt;
                F[STATE_VY][STATE_D2] =  GRAVITY*R[2][0]*dt;
                F[STATE_VZ][STATE_D2] =  0;

                F[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                F[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
                F[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

                F[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
                F[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                F[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

                F[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
                F[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
                F[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

             }

            static void addProcessNoise(const float dt, const uint32_t msec_curr,
                    float P[STATE_DIM][STATE_DIM]) 
            {
                const float noise[STATE_DIM] = {
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                ekf_addCovarianceNoise(noise, P);

                ekf_enforceSymmetry(P);
            }

            static void updateWithRange(
                    const ZRangerFilter & zrfilter,
                    const float R[3][3],
                    float x[STATE_DIM],
                    float P[STATE_DIM][STATE_DIM])
            {
                // Updates the filter with a measured distance in the zb direction using the
                float h[STATE_DIM] = {};

                // Only update the filter if the measurement is reliable 
                // (\hat{h} -> infty when R[2][2] -> 0)
                if (fabs(R[2][2]) > 0.1f && R[2][2] > 0) {
                    const auto angle = max(0, fabsf(acosf(R[2][2])) -
                            Num::DEG2RAD * (15.0f / 2));
                    const auto predictedDistance = x[STATE_Z] / cosf(angle);
                    const auto measuredDistance = zrfilter.distance_m;

                    // This just acts like a gain for the sensor model. Further
                    // updates are done in the scalar update function below
                    h[STATE_Z] = 1 / cosf(angle); 

                    ekf_updateWithScalar(h, measuredDistance-predictedDistance,
                            zrfilter.stdev, x, P);
                }
            }
 
            static void updateWithFlow(
                    const OpticalFlowFilter & offilter,
                    const ThreeAxis & gyro,
                    const float r22,
                    float x[STATE_DIM],
                    float P[STATE_DIM][STATE_DIM])
            {
                // [pixels] (same in x and y)
                const float Npix = 35.0;                      

                //float thetapix = Num::DEG2RAD * 4.0f;
                // [rad]    (same in x and y)
                // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
                // corresponding ground length
                const float thetapix = 0.71674f;

                //~~~ Body rates ~~~
                const auto omegax_b = gyro.x * Num::DEG2RAD;
                const auto omegay_b = gyro.y * Num::DEG2RAD;

                const auto dx_g = x[STATE_VX];
                const auto dy_g = x[STATE_VY];

                // Saturate elevation in prediction and correction to avoid singularities
                const auto z_g  = max(x[STATE_Z], 0.1);

                // ~~~ X velocity prediction and update ~~~
                // predicts the number of accumulated pixels in the x-direction
                float hx[STATE_DIM] = {};
                const auto predictedNX = (offilter.dt * Npix / thetapix ) * 
                    ((dx_g * r22 / z_g) - omegay_b);
                const auto measuredNX = offilter.dpixelx*FLOW_RESOLUTION;

                // derive measurement equation with respect to dx (and z?)
                hx[STATE_Z] = (Npix * offilter.dt / thetapix) * 
                    ((r22 * dx_g) / (-z_g * z_g));
                hx[STATE_VX] = (Npix * offilter.dt / thetapix) * 
                    (r22 / z_g);

                //First update
                ekf_updateWithScalar(hx, (measuredNX-predictedNX), 
                        offilter.stdDevX*FLOW_RESOLUTION, x, P);

                // ~~~ Y velocity prediction and update ~~~
                float hy[STATE_DIM] = {};
                const auto predictedNY = (offilter.dt * Npix / thetapix ) * 
                    ((dy_g * r22 / z_g) + omegax_b);
                const auto measuredNY = offilter.dpixely*FLOW_RESOLUTION;

                // derive measurement equation with respect to dy (and z?)
                hy[STATE_Z] = (Npix * offilter.dt / thetapix) * 
                    ((r22 * dy_g) / (-z_g * z_g));
                hy[STATE_VY] = (Npix * offilter.dt / thetapix) * (r22 / z_g);

                // Second update
                ekf_updateWithScalar(hy, (measuredNY-predictedNY),
                        offilter.stdDevY*FLOW_RESOLUTION, x, P);
            }

            static void ekf_init(
                    float x[STATE_DIM], float P[STATE_DIM][STATE_DIM]) 
            {
                for (int i=0; i< STATE_DIM; i++) {

                    x[i] = 0;

                    for (int j=0; j < STATE_DIM; j++) {
                        P[i][j] = 0; 
                    }
                }
            }

            static void ekf_enforceSymmetry(float P[STATE_DIM][STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        P[i][j] = P[j][i] =
                            ekf_pval(i, j, 0.5*P[i][j] + 0.5*P[j][i]);
                    }
                }
            }

            static void ekf_updateWithScalar(
                    const float * h,
                    const float error,
                    const float stdMeasNoise,
                    float x[STATE_DIM],
                    float P[STATE_DIM][STATE_DIM])
            {
                static float G[STATE_DIM];

                const auto R = stdMeasNoise*stdMeasNoise;

                float PHt[STATE_DIM] = {};
                dot(P, h, PHt); // PH'

                float HPHR = R; // HPH' + R
                for (size_t i=0; i<STATE_DIM; i++) { 
                    HPHR += h[i] * PHt[i]; 
                }

                for (size_t i=0; i<STATE_DIM; i++) {
                    G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                }

                float GH[STATE_DIM][STATE_DIM] = {};
                float GH_I[STATE_DIM][STATE_DIM] = {};
                float GH_I_P[STATE_DIM][STATE_DIM] = {};

                outer(G, h, GH);

                // GH - I
                for (size_t i=0; i<STATE_DIM; i++) { 
                    GH[i][i] -= 1; 
                }

                // (GH - I)'
                trans(GH, GH_I);

                // (GH - I)*P
                dot(GH, P, GH_I_P); 

                // (GH - I)*P*(GH - I)'
                dot(GH_I_P, GH_I, P);

                // add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<STATE_DIM; i++) {

                    x[i] += G[i] * error; // state update

                    for (int j=i; j<STATE_DIM; j++) {

                        const auto v = G[i] * R * G[j];

                        // add measurement noise
                        P[i][j] = P[j][i] =
                            ekf_pval(i, j, 0.5*P[i][j] + 0.5*P[j][i] + v); 
                    }
                }
            }

            static void ekf_addCovarianceNoise(const float * noise,
                    float P[STATE_DIM][STATE_DIM])
            {
                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    P[k][k] += noise[k] * noise[k];
                }
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

            static auto ekf_pval(
                    const int i, const int j, const float pval) -> float
            {
                return
                    isnan(pval) || pval > MAX_COVARIANCE ? MAX_COVARIANCE :
                    i==j && pval < MIN_COVARIANCE ? MIN_COVARIANCE :
                    pval;
            }

            // C = A * B
            static void dot(
                    const float A[STATE_DIM][STATE_DIM],
                    const float B[STATE_DIM][STATE_DIM],
                    float C[STATE_DIM][STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        C[i][j] = 0;
                        for (int k=0; k<STATE_DIM; ++k) {
                            C[i][j] += A[i][k] * B[k][j];
                        }
                    }
                }
            }

            // y = A * x
            static void dot(
                    const float A[STATE_DIM][STATE_DIM],
                    const float x[STATE_DIM],
                    float y[STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; i++) {
                    y[i] = 0; 
                    for (int j=0; j<STATE_DIM; j++) {
                        y[i] += A[i][j] * x[j];
                    }
                }
            }

            // A = x * y
            static void outer(
                    const float x[STATE_DIM],
                    const float y[STATE_DIM],
                    float C[STATE_DIM][STATE_DIM])
            {
                for (size_t i=0; i<STATE_DIM; i++) {
                    for (size_t j=0; j<STATE_DIM; j++) {
                        C[i][j] = x[i] * y[j];
                    }
                }
            }

            // At = A^T
            static void trans(
                    const float A[STATE_DIM][STATE_DIM],
                    float At[STATE_DIM][STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        At[i][j] = A[j][i];
                    }
                }
            }
    };

}
