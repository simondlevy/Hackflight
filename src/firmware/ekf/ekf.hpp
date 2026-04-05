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

        public:

            EKF& operator=(const EKF& other) = default;
 
            EKF()
            {
                ekf_init();

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
                const float pinit[STATE_DIM] = {

                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                ekf_addCovarianceNoise(pinit);

                _didPredict = false;
                _didUpdateWithFlowDeck = false;
                _lastPredictionMs = 0;
                _lastProcessNoiseUpdateMs = 0;
            }

            void predict(const uint32_t msec_curr, bool isFlying) 
            {
                _accelSubSampler = ThreeAxisSubSampler::finalize(_accelSubSampler);
                _gyroSubSampler = ThreeAxisSubSampler::finalize(_gyroSubSampler);

                const float dt = (msec_curr - _lastPredictionMs) / 1000.0f;

                const ThreeAxis * accel = &_accelSubSampler.subSample;
                const ThreeAxis * gyro = &_gyroSubSampler.subSample;

                const float d0 = gyro->x*dt/2;
                const float d1 = gyro->y*dt/2;
                const float d2 = gyro->z*dt/2;

                const float vx = _x[STATE_VX];
                const float vy = _x[STATE_VY];
                const float vz = _x[STATE_VZ];

                // The linearized Jacobean matrix
                static float F[STATE_DIM][STATE_DIM];

                // position
                F[STATE_Z][STATE_Z] = 1;

                // position from body-frame velocity
                F[STATE_Z][STATE_VX] = _r20*dt;

                F[STATE_Z][STATE_VY] = _r21*dt;

                F[STATE_Z][STATE_VZ] = _r22*dt;

                // position from attitude error
                F[STATE_Z][STATE_D0] = (vy*_r22 - vz*_r21)*dt;

                F[STATE_Z][STATE_D1] = (-vx*_r22 + vz*_r20)*dt;

                F[STATE_Z][STATE_D2] = (vx*_r21 - vy*_r20)*dt;

                // body-frame velocity from body-frame velocity
                F[STATE_VX][STATE_VX] = 1; //drag negligible
                F[STATE_VY][STATE_VX] =-gyro->z*dt;
                F[STATE_VZ][STATE_VX] = gyro->y*dt;

                F[STATE_VX][STATE_VY] = gyro->z*dt;
                F[STATE_VY][STATE_VY] = 1; //drag negligible
                F[STATE_VZ][STATE_VY] =-gyro->x*dt;

                F[STATE_VX][STATE_VZ] =-gyro->y*dt;
                F[STATE_VY][STATE_VZ] = gyro->x*dt;
                F[STATE_VZ][STATE_VZ] = 1; //drag negligible

                // body-frame velocity from attitude error
                F[STATE_VX][STATE_D0] =  0;
                F[STATE_VY][STATE_D0] = -GRAVITY*_r22*dt;
                F[STATE_VZ][STATE_D0] =  GRAVITY*_r21*dt;

                F[STATE_VX][STATE_D1] =  GRAVITY*_r22*dt;
                F[STATE_VY][STATE_D1] =  0;
                F[STATE_VZ][STATE_D1] = -GRAVITY*_r20*dt;

                F[STATE_VX][STATE_D2] = -GRAVITY*_r21*dt;
                F[STATE_VY][STATE_D2] =  GRAVITY*_r20*dt;
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

                // P_k = F_{k-1} P_{k-1} F^T_{k-1}
                device_predict(F, _P);

                const float dt2 = dt * dt;

                // keep previous time step's state for the update
                const float tmpSPX = _x[STATE_VX];
                const float tmpSPY = _x[STATE_VY];
                const float tmpSPZ = _x[STATE_VZ];

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _x[STATE_VX] * dt + (isFlying ? 0 : accel->x * dt2 / 2.0f);
                const float dy = _x[STATE_VY] * dt + (isFlying ? 0 : accel->y * dt2 / 2.0f);

                // thrust can only be produced in the body's Z direction
                const float dz = _x[STATE_VZ] * dt + accel->z * dt2 / 2.0f; 

                // position update
                _x[STATE_Z] += _r20 * dx + _r21 * dy + _r22 * dz - 
                    GRAVITY * dt2 / 2.0f;

                const float accelx = isFlying ? 0 : accel->x;
                const float accely = isFlying ? 0 : accel->y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame

                _x[STATE_VX] += dt * (accelx + gyro->z * tmpSPY - gyro->y * tmpSPZ
                        - GRAVITY * _r20);

                _x[STATE_VY] += dt * (accely - gyro->z * tmpSPX + gyro->x * tmpSPZ
                        - GRAVITY * _r21);

                _x[STATE_VZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY
                        - GRAVITY * _r22);

                // attitude update (rotate by gyroscope), we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const float dtwx = dt*gyro->x;
                const float dtwy = dt*gyro->y;
                const float dtwz = dt*gyro->z;

                // compute the quaternion values in [w,x,y,z] order
                const float angle = device_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + Num::EPSILON;
                const float ca = device_cos(angle/2.0f);
                const float sa = device_sin(angle/2.0f);
                const float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

                // rotate the vehicle's attitude by the delta quaternion vector computed above

                float tmpq0 = dq[0]*_q.w - dq[1]*_q.x - dq[2]*_q.y - dq[3]*_q.z;
                float tmpq1 = dq[1]*_q.w + dq[0]*_q.x + dq[3]*_q.y - dq[2]*_q.z;
                float tmpq2 = dq[2]*_q.w - dq[3]*_q.x + dq[0]*_q.y + dq[1]*_q.z;
                float tmpq3 = dq[3]*_q.w + dq[2]*_q.x - dq[1]*_q.y + dq[0]*_q.z;

                if (!isFlying) {

                    const float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                    tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION;
                    tmpq1 = keep * tmpq1; 
                    tmpq2 = keep * tmpq2; 
                    tmpq3 = keep * tmpq3; 
                }

                // normalize and store the result
                const float norm = device_sqrt(
                        tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + Num::EPSILON;

                _q.w = tmpq0/norm; 
                _q.x = tmpq1/norm; 
                _q.y = tmpq2/norm; 
                _q.z = tmpq3/norm;

                _didPredict = true;
                _lastPredictionMs = msec_curr;

            } // predict

            void update(const ImuFilter::Data & imudata,
                    const uint32_t msec_curr)
            {
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

            //////////////////////////////////////////////////////////////////

            void addProcessNoise(const uint32_t msec_curr) 
            {
                float dt = (msec_curr - _lastProcessNoiseUpdateMs) / 1000.0f;

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

                    ekf_addCovarianceNoise(noise);

                    ekf_enforceSymmetry();

                    _lastProcessNoiseUpdateMs = msec_curr;
                }
            }

            void updateWithFlow(const OpticalFlowFilter & offilter)
            {
                const ThreeAxis *gyro = &_gyroLatest;

                // [pixels] (same in x and y)
                float Npix = 35.0;                      

                //float thetapix = Num::DEG2RAD * 4.0f;
                // [rad]    (same in x and y)
                // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
                // corresponding ground length
                float thetapix = 0.71674f;

                //~~~ Body rates ~~~
                // TODO check if this is feasible or if some filtering has to be done
                float omegax_b = gyro->x * Num::DEG2RAD;
                float omegay_b = gyro->y * Num::DEG2RAD;


                float dx_g = _x[STATE_VX];
                float dy_g = _x[STATE_VY];
                float z_g = 0.0;
                // Saturate elevation in prediction and correction to avoid singularities
                if ( _x[STATE_Z] < 0.1f ) {
                    z_g = 0.1;
                } else {
                    z_g = _x[STATE_Z];
                }

                // ~~~ X velocity prediction and update ~~~
                // predicts the number of accumulated pixels in the x-direction
                float hx[STATE_DIM] = {};
                _predictedNX = (offilter.dt * Npix / thetapix ) * 
                    ((dx_g * _r22 / z_g) - omegay_b);
                _measuredNX = offilter.dpixelx*FLOW_RESOLUTION;

                // derive measurement equation with respect to dx (and z?)
                hx[STATE_Z] = (Npix * offilter.dt / thetapix) * 
                    ((_r22 * dx_g) / (-z_g * z_g));
                hx[STATE_VX] = (Npix * offilter.dt / thetapix) * 
                    (_r22 / z_g);

                //First update
                ekf_updateWithScalar(hx, (_measuredNX-_predictedNX), 
                        offilter.stdDevX*FLOW_RESOLUTION);

                // ~~~ Y velocity prediction and update ~~~
                float hy[STATE_DIM] = {};
                _predictedNY = (offilter.dt * Npix / thetapix ) * 
                    ((dy_g * _r22 / z_g) + omegax_b);
                _measuredNY = offilter.dpixely*FLOW_RESOLUTION;

                // derive measurement equation with respect to dy (and z?)
                hy[STATE_Z] = (Npix * offilter.dt / thetapix) * 
                    ((_r22 * dy_g) / (-z_g * z_g));
                hy[STATE_VY] = (Npix * offilter.dt / thetapix) * (_r22 / z_g);

                // Second update
                ekf_updateWithScalar(hy, (_measuredNY-_predictedNY),
                        offilter.stdDevY*FLOW_RESOLUTION);
            }

            void updateWithRange(const ZRangerFilter & zrfilter)
            {
                // Updates the filter with a measured distance in the zb direction using the
                float h[STATE_DIM] = {};

                // Only update the filter if the measurement is reliable 
                // (\hat{h} -> infty when R[2][2] -> 0)
                if (fabs(_r22) > 0.1f && _r22 > 0) {
                    float angle = 
                        fabsf(acosf(_r22)) - 
                        Num::DEG2RAD * (15.0f / 2.0f);
                    if (angle < 0.0f) {
                        angle = 0.0f;
                    }
                    float predictedDistance = _x[STATE_Z] / cosf(angle);
                    float measuredDistance = zrfilter.distance_m;

                    // This just acts like a gain for the sensor model. Further
                    // updates are done in the scalar update function below
                    h[STATE_Z] = 1 / cosf(angle); 

                    ekf_updateWithScalar(h, measuredDistance-predictedDistance,
                            zrfilter.stdev);
                }
            }

            void finalize()
            {
                // Incorporate the attitude error (Kalman filter state) with the attitude
                const float v0 = _x[STATE_D0];
                const float v1 = _x[STATE_D1];
                const float v2 = _x[STATE_D2];

                // Move attitude error into attitude if any of the angle errors are
                // large enough
                if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) >
                            0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 &&
                                fabsf(v2) < 10)) {

                    const float angle = device_sqrt(v0*v0 + v1*v1 + v2*v2) + Num::EPSILON;
                    const float ca = device_cos(angle / 2.0f);
                    const float sa = device_sin(angle / 2.0f);
                    const auto dq = Quaternion(
                            ca, sa*v0/angle, sa*v1/angle, sa*v2/angle);

                    // Rotate the vehicle's attitude by the delta quaternion vector
                    // computed above
                    const auto tmpq = Quaternion(
                            dq.w*_q.w - dq.x*_q.x - dq.y*_q.y - dq.z*_q.z,
                            dq.x*_q.w + dq.w*_q.x + dq.z*_q.y - dq.y*_q.z,
                            dq.y*_q.w - dq.z*_q.x + dq.w*_q.y + dq.x*_q.z,
                            dq.z*_q.w + dq.y*_q.x - dq.x*_q.y + dq.w*_q.z);

                    // normalize and store the result
                    float norm = device_sqrt(
                            tmpq.w * tmpq.w + tmpq.x * tmpq.x + tmpq.y * tmpq.y + 
                            tmpq.z * tmpq.z) + Num::EPSILON;
                    _q.w = tmpq.w / norm;
                    _q.x = tmpq.x / norm;
                    _q.y = tmpq.y / norm;
                    _q.z = tmpq.z / norm;
                }

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and accel

                _r00 = _q.w * _q.w + _q.x * _q.x - _q.y * _q.y - _q.z * _q.z;
                _r01 = 2 * _q.x * _q.y - 2 * _q.w * _q.z;
                _r02 = 2 * _q.x * _q.z + 2 * _q.w * _q.y;
                _r10 = 2 * _q.x * _q.y + 2 * _q.w * _q.z;
                _r11 = _q.w * _q.w - _q.x * _q.x + _q.y * _q.y - _q.z * _q.z;
                _r12 = 2 * _q.y * _q.z - 2 * _q.w * _q.x;
                _r20 = 2 * _q.x * _q.z - 2 * _q.w * _q.y;
                _r21 = 2 * _q.y * _q.z + 2 * _q.w * _q.x;
                _r22 = _q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z;

                // reset the attitude error
                _x[STATE_D0] = 0;
                _x[STATE_D1] = 0;
                _x[STATE_D2] = 0;

                ekf_enforceSymmetry();

            } // finalize

            void ekf_init()
            {
                for (int i=0; i< STATE_DIM; i++) {

                    _x[i] = 0;

                    for (int j=0; j < STATE_DIM; j++) {
                        _P[i][j] = 0; 
                    }
                }
            }

            void ekf_addCovarianceNoise(const float * noise)
            {
                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    _P[k][k] += noise[k] * noise[k];
                }
            }

            void ekf_enforceSymmetry()
            {
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        ekf_pset(i, j, 0.5 * _P[i][j] + 0.5 * _P[j][i]);
                    }
                }
            }

            void ekf_updateWithScalar(const float * h, const float error, const float stdMeasNoise)
            {
                static float G[STATE_DIM];

                const auto R = stdMeasNoise*stdMeasNoise;

                device_update_with_scalar(_P, h, error, R, _x, G);

                // add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        float v = G[i] * R * G[j];

                        // add measurement noise
                        ekf_pset(i, j, 0.5 * _P[i][j] + 0.5 * _P[j][i] + v); 
                    }
                }
            }

            void ekf_pset(const uint8_t i, const uint8_t j, const float pval)
            {
                if (isnan(pval) || pval > MAX_COVARIANCE) {
                    _P[i][j] = _P[j][i] = MAX_COVARIANCE;
                } else if ( i==j && pval < MIN_COVARIANCE ) {
                    _P[i][j] = _P[j][i] = MIN_COVARIANCE;
                } else {
                    _P[i][j] = _P[j][i] = pval;
                }
            }

            // Hardware-dependent --------------------------------------------

            static void device_predict(
                    const float F[STATE_DIM][STATE_DIM],
                    float P[STATE_DIM][STATE_DIM]);

            static void device_update_with_scalar(
                    const float P[STATE_DIM][STATE_DIM],
                    const float * h,
                    const float error,
                    const float R,
                    float x[STATE_DIM],
                    float G[STATE_DIM]);

            static float device_cos(const float x);
            static float device_sin(const float x);
            static float device_sqrt(const float in);
    };

}
