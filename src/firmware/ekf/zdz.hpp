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

#include <array>

#include <firmware/ekf/quaternion.hpp>
#include <firmware/ekf/rotation.hpp>
#include <firmware/ekf/three_axis_subsampler.hpp>
#include <firmware/imu/filter.hpp>
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

            typedef std::array<float, STATE_DIM*STATE_DIM> matrix;

            typedef std::array<float, STATE_DIM> vector;

            class Core {

                public:

                    // State vector
                    vector x;

                    // Covariance matrix
                    matrix P;

                    Core() = default;

                    Core& operator=(const Core& other) = default;

                    Core(const vector & x, const matrix & P) 
                        : x(x), P(P) {}
            };

            Core core;

        public:

            EKF& operator=(const EKF& other) = default;

            EKF()
            {
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

                core.P = addCovarianceNoise(matrix(), pinit);

                didUpdateWithZRanger = false;
                lastProcessNoiseUpdateMsec = 0;

                didPredict = false;
                lastPredictionMsec = 0;
            }

            EKF(
                    const Core & core,
                    const Quaternion & q,
                    const ThreeAxis & gyroLatest,
                    const ThreeAxisSubSampler & accelSubSampler,
                    const ThreeAxisSubSampler & gyroSubSampler,
                    const Rotation & R,
                    const bool didUpdateWithZRanger,
                    const ZRangerFilter & zrangerFilterLatest,
                    const uint32_t lastProcessNoiseUpdateMsec,
                    const bool didPredict,
                    const uint32_t lastPredictionMsec)
                :
                    core(core),
                    q(q),
                    gyroLatest(gyroLatest),
                    accelSubSampler(accelSubSampler),
                    gyroSubSampler(gyroSubSampler),
                    R(R),
                    didUpdateWithZRanger(didUpdateWithZRanger),
                    zrangerFilterLatest(zrangerFilterLatest),
                    lastProcessNoiseUpdateMsec(lastProcessNoiseUpdateMsec),
                    didPredict(didPredict),
                    lastPredictionMsec(lastPredictionMsec) {}

            static auto predict(const EKF & ekf, const uint32_t msec_curr,
                    const bool isFlying) -> EKF
            {
                const auto accelSubSampler =
                    ThreeAxisSubSampler::finalize(ekf.accelSubSampler);

                const auto gyroSubSampler =
                    ThreeAxisSubSampler::finalize(ekf.gyroSubSampler);

                const auto dt = (msec_curr - ekf.lastPredictionMsec) / 1000.f;

                const auto accel = accelSubSampler.subSample;
                const auto gyro = gyroSubSampler.subSample;

                // The linearized Jacobean matrix
                const auto F = makeJacobian(dt, gyro, ekf.core.x, ekf.R);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
                const auto P = dot(dot(F, ekf.core.P), trans(F));

                const auto dt2 = dt * dt;

                // keep previous time step's state for the update
                const auto tmpSPX = ekf.core.x[STATE_VX];
                const auto tmpSPY = ekf.core.x[STATE_VY];
                const auto tmpSPZ = ekf.core.x[STATE_VZ];

                // position updates in the body frame (will be rotated to inertial frame)
                const auto dx = ekf.core.x[STATE_VX] * dt + (isFlying ? 0 : accel.x * dt2 / 2);
                const auto dy = ekf.core.x[STATE_VY] * dt + (isFlying ? 0 : accel.y * dt2 / 2);

                // thrust can only be produced in the body's Z direction
                const auto dz = ekf.core.x[STATE_VZ] * dt + accel.z * dt2 / 2; 

                const auto accelx = isFlying ? 0 : accel.x;
                const auto accely = isFlying ? 0 : accel.y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                auto x = vector();
                x[STATE_Z] = ekf.core.x[STATE_Z] + ekf.R.zx * dx + ekf.R.zy * dy +
                    ekf.R.zz * dz - GRAVITY * dt2 / 2;

                x[STATE_VX] = ekf.core.x[STATE_VX] + dt * (accelx + gyro.z * tmpSPY -
                        gyro.y * tmpSPZ - GRAVITY * ekf.R.zx);

                x[STATE_VY] = ekf.core.x[STATE_VY] + dt * (accely - gyro.z * tmpSPX +
                        gyro.x * tmpSPZ - GRAVITY * ekf.R.zy);

                x[STATE_VZ] = ekf.core.x[STATE_VZ] + dt * (accel.z + gyro.y * tmpSPX -
                        gyro.x * tmpSPY - GRAVITY * ekf.R.zz);

                x[STATE_D0] = ekf.core.x[STATE_D0];
                x[STATE_D1] = ekf.core.x[STATE_D1];
                x[STATE_D2] = ekf.core.x[STATE_D2];

                // Attitude update (rotate by gyroscope): we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const auto dtw = gyro * dt;

                // compute the quaternion values in [w,x,y,z] order
                auto tmpq = rotate(dtw, ekf.q);

                const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                const auto newtmpq = isFlying ? tmpq : 
                    Quaternion(
                            tmpq.w = keep * tmpq.w + ROLLPITCH_ZERO_REVERSION,
                            tmpq.x = keep * tmpq.x, 
                            tmpq.y = keep * tmpq.y, 
                            tmpq.z = keep * tmpq.z); 

                // normalize and store the result
                const auto q = newtmpq / Quaternion::l2norm(newtmpq);

                return EKF(
                        Core(x, P),
                        q,
                        ekf.gyroLatest,
                        accelSubSampler,
                        gyroSubSampler,
                        ekf.R,
                        ekf.didUpdateWithZRanger,
                        ekf.zrangerFilterLatest,
                        ekf.lastProcessNoiseUpdateMsec,
                        true, // didPredict,
                        msec_curr);  // lastPredictionMsec

            } // predict

            static auto update(
                    const EKF & ekf,
                    const ImuFilter::Data & imudata,
                    const uint32_t msec_curr) -> EKF
            {
                const auto dt =
                    (msec_curr - ekf.lastProcessNoiseUpdateMsec) / 1000.0f;

                const auto dtpositive = dt > 0;

                const float noise[STATE_DIM] = {
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                const auto lastProcessNoiseUpdateMsec =
                    dtpositive ? msec_curr : ekf.lastProcessNoiseUpdateMsec;

                const auto accelSubSampler = ThreeAxisSubSampler::accumulate(
                        ekf.accelSubSampler, imudata.accelGs);

                const auto gyroSubSampler = ThreeAxisSubSampler::accumulate(
                        ekf.gyroSubSampler, imudata.gyroDps);

                const auto gyroLatest = imudata.gyroDps;

                const auto rzz = ekf.R.zz;

                const auto rangeok = fabs(rzz) > 0.1 && rzz > 0; 

                const auto coreWithNoise = dtpositive ?
                    Core(ekf.core.x, 
                            enforceSymmetry(addCovarianceNoise(ekf.core.P, noise))) :
                    ekf.core;

                const auto coreWithRange = rangeok && ekf.didUpdateWithZRanger ?
                    updateWithRange(
                            coreWithNoise, ekf.zrangerFilterLatest, rzz) :
                    coreWithNoise;

                const auto ready = ekf.didUpdateWithZRanger || ekf.didPredict;

                // Incorporate the attitude error (Kalman filter state) with the attitude
                const auto v = ThreeAxis(
                        ekf.core.x[STATE_D0], ekf.core.x[STATE_D1], ekf.core.x[STATE_D2]);

                // reset the attitude error
                const auto x = vector{
                    coreWithRange.x[0],
                    coreWithRange.x[1],
                    coreWithRange.x[2],
                    coreWithRange.x[3],
                    0, 0, 0};

                const auto P = enforceSymmetry(coreWithRange.P);

                const auto q = ready &&
                    (bigenough(v.x) || bigenough(v.y) || bigenough(v.z)) &&
                    smallenough(v.x) && smallenough(v.y) && smallenough(v.z) ?
                    ekf.q / Quaternion::l2norm(rotate(v, ekf.q)) : ekf.q;

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and accel
                const auto R = ready ?
                    Rotation(
                            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
                            2 * q.x * q.y - 2 * q.w * q.z,
                            2 * q.x * q.z + 2 * q.w * q.y,
                            2 * q.x * q.y + 2 * q.w * q.z,
                            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
                            2 * q.y * q.z - 2 * q.w * q.x,
                            2 * q.x * q.z - 2 * q.w * q.y,
                            2 * q.y * q.z + 2 * q.w * q.x,
                            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) : ekf.R;

                const auto core = ready ? Core(x, P) : coreWithRange;

                return EKF(
                        core,
                        q,
                        gyroLatest,
                        accelSubSampler,
                        gyroSubSampler,
                        R,
                        false, // didUpdateWithZRanger
                        ekf.zrangerFilterLatest,
                        lastProcessNoiseUpdateMsec,
                        false, // didPredict
                        ekf.lastPredictionMsec);

            } // update

            static auto update(
                    const EKF &ekf,
                    const ZRangerFilter & zrfilter) -> EKF
            {
                return EKF(
                        ekf.core,
                        ekf.q,
                        ekf.gyroLatest,
                        ekf.accelSubSampler,
                        ekf.gyroSubSampler,
                        ekf.R,
                        true, // didUpdateWithZRanger,
                        zrfilter,
                        ekf.lastProcessNoiseUpdateMsec,
                        ekf. didPredict,
                        ekf. lastPredictionMsec);
            }

            static auto getVehicleState(const EKF & ekf) -> VehicleState
            {
                const auto x = ekf.core.x;

                const auto dx =
                    ekf.R.xx*x[STATE_VX] +
                    ekf.R.xy*x[STATE_VY] +
                    ekf.R.xz*x[STATE_VZ];

                // make right positive
                const auto dy = -(
                        ekf.R.yx*x[STATE_VX] +
                        ekf.R.yy*x[STATE_VY] +
                        ekf.R.yz*x[STATE_VZ]); 

                const auto z = x[STATE_Z];

                const auto dz =
                    ekf.R.zx*x[STATE_VX] +
                    ekf.R.zy*x[STATE_VY] +
                    ekf.R.zz*x[STATE_VZ];

                const auto q0 = ekf.q.w;
                const auto q1 = ekf.q.x;
                const auto q2 = ekf.q.y;
                const auto q3 = ekf.q.z;

                const auto phi = Num::RAD2DEG * atan2f(2*(q2*q3+q0* q1) ,
                        q0*q0 - q1*q1 - q2*q2 + q3*q3);

                const auto dphi = ekf.gyroLatest.x;

                const auto theta = Num::RAD2DEG * asinf(-2*(q1*q3 - q0*q2));

                const auto dtheta = ekf.gyroLatest.y;

                const auto psi = Num::RAD2DEG * atan2f(2*(q1*q2+q0* q3),
                        q0*q0 + q1*q1 - q2*q2 - q3*q3); 

                const auto dpsi = ekf.gyroLatest.z;

                // Return psi/dpsi nose-right positive
                return VehicleState(
                        dx, dy, z, dz, phi, dphi, theta, dtheta, -psi, -dpsi);
            }

        private:

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Quaternion q;

            ThreeAxis gyroLatest;

            ThreeAxisSubSampler accelSubSampler = ThreeAxisSubSampler(GRAVITY);
            ThreeAxisSubSampler gyroSubSampler = ThreeAxisSubSampler(Num::DEG2RAD);

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Rotation R;

            bool didUpdateWithZRanger;

            ZRangerFilter zrangerFilterLatest;

            uint32_t lastProcessNoiseUpdateMsec;

            bool didPredict;
            uint32_t lastPredictionMsec;

            static auto makeJacobian(
                    const float dt,
                    const ThreeAxis & gyro,
                    const vector x,
                    const Rotation &R) -> matrix
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x[STATE_VX];
                const auto vy = x[STATE_VY];
                const auto vz = x[STATE_VZ];

                const size_t N = STATE_DIM;

                auto F = matrix();

                // position
                F[STATE_Z*N+STATE_Z] = 1;
                F[STATE_Z*N+STATE_VX] = R.zx*dt;
                F[STATE_Z*N+STATE_VY] = R.zy*dt;
                F[STATE_Z*N+STATE_VZ] = R.zz*dt;
                F[STATE_Z*N+STATE_D0] = (vy*R.zz - vz*R.zy)*dt;
                F[STATE_Z*N+STATE_D1] = (-vx*R.zz + vz*R.zx)*dt;
                F[STATE_Z*N+STATE_D2] = (vx*R.zy - vy*R.zx)*dt;

                F[STATE_VX*N+STATE_Z] = 0; 
                F[STATE_VX*N+STATE_VX] = 1; 
                F[STATE_VX*N+STATE_VY] = gyro.z*dt;
                F[STATE_VX*N+STATE_VZ] =-gyro.y*dt;
                F[STATE_VX*N+STATE_D0] =  0;
                F[STATE_VX*N+STATE_D1] =  GRAVITY*R.zz*dt;
                F[STATE_VX*N+STATE_D2] = -GRAVITY*R.zy*dt;

                F[STATE_VY*N+STATE_Z] = 0; 
                F[STATE_VY*N+STATE_VX] =-gyro.z*dt;
                F[STATE_VY*N+STATE_VY] = 1; 
                F[STATE_VY*N+STATE_VZ] = gyro.x*dt;
                F[STATE_VY*N+STATE_D0] = -GRAVITY*R.zz*dt;
                F[STATE_VY*N+STATE_D1] =  0;
                F[STATE_VY*N+STATE_D2] =  GRAVITY*R.zx*dt;

                F[STATE_VZ*N+STATE_Z] = 0; 
                F[STATE_VZ*N+STATE_VX] = gyro.y*dt;
                F[STATE_VZ*N+STATE_VY] =-gyro.x*dt;
                F[STATE_VZ*N+STATE_VZ] = 1; 
                F[STATE_VZ*N+STATE_D0] =  GRAVITY*R.zy*dt;
                F[STATE_VZ*N+STATE_D1] = -GRAVITY*R.zx*dt;
                F[STATE_VZ*N+STATE_D2] =  0;

                F[STATE_D0*N+STATE_Z] = 0; 
                F[STATE_D0*N+STATE_VX] = 0; 
                F[STATE_D0*N+STATE_VX] = 0; 
                F[STATE_D0*N+STATE_VZ] = 0; 
                F[STATE_D0*N+STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                F[STATE_D0*N+STATE_D1] =  d2 + d0*d1/2;
                F[STATE_D0*N+STATE_D2] = -d1 + d0*d2/2;

                F[STATE_D1*N+STATE_Z] = 0; 
                F[STATE_D1*N+STATE_VX] = 0; 
                F[STATE_D1*N+STATE_VX] = 0; 
                F[STATE_D1*N+STATE_VZ] = 0; 
                F[STATE_D1*N+STATE_D0] = -d2 + d0*d1/2;
                F[STATE_D1*N+STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                F[STATE_D1*N+STATE_D2] =  d0 + d1*d2/2;

                F[STATE_D2*N+STATE_Z] = 0; 
                F[STATE_D2*N+STATE_VX] = 0; 
                F[STATE_D2*N+STATE_VX] = 0; 
                F[STATE_D2*N+STATE_VZ] = 0; 
                F[STATE_D2*N+STATE_D0] =  d1 + d0*d2/2;
                F[STATE_D2*N+STATE_D1] = -d0 + d1*d2/2;
                F[STATE_D2*N+STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

                return F;
            }

            static auto updateWithRange(
                    const Core & core,
                    const ZRangerFilter & zrfilter,
                    const float rzz) -> Core
            {

                const auto angle = max(0, fabsf(acosf(rzz)) -
                        Num::DEG2RAD * (15.0f / 2));
                const auto predictedDistance = core.x[STATE_Z] / cosf(angle);
                const auto measuredDistance = zrfilter.distance_m;

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                const vector h = {1 / cosf(angle), 0, 0, 0, 0, 0, 0 };

                return updateWithScalar(core, h,
                        measuredDistance-predictedDistance,
                        zrfilter.stdev);
            }

            static auto updateWithScalar(
                    const Core & core,
                    const vector & h,
                    const float error,
                    const float stdMeasNoise) -> Core
            {
                const auto R = stdMeasNoise*stdMeasNoise;

                const auto PHt = dot(core.P, h); // PH'

                float HPHR = R; // HPH' + R
                for (size_t i=0; i<STATE_DIM; i++) { 
                    HPHR += h[i] * PHt[i]; 
                }

                vector G;
                for (size_t i=0; i<STATE_DIM; i++) {
                    G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                }

                auto GH = outer(G, h);

                // GH - I
                for (size_t i=0; i<STATE_DIM; i++) { 
                    GH[i*STATE_DIM+i] -= 1; 
                }

                // (GH - I)'
                const auto GH_I = trans(GH);

                // (GH - I)*P
                const auto GH_I_P = dot(GH, core.P); 

                // (GH - I)*P*(GH - I)'
                auto P = dot(GH_I_P, GH_I);

                // State update
                auto x = vector();
                for (int i=0; i<STATE_DIM; i++) {
                    x[i] = core.x[i] + G[i] * error; 
                }

                // Add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        const auto v = G[i] * R * G[j];

                        // add measurement noise
                        P[i*STATE_DIM+j] = P[j*STATE_DIM+i] =
                            get_pval(i, j, 0.5*P[i*STATE_DIM+j] + 0.5*P[j*STATE_DIM+i] + v,
                                    MIN_COVARIANCE, MAX_COVARIANCE); 
                    }
                }

                return Core(x, P);
            }

            static auto enforceSymmetry(const matrix & P) -> matrix
            {
                auto Pnew = P;

                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        Pnew[i*STATE_DIM+j] = Pnew[j*STATE_DIM+i] =
                            get_pval(i, j,
                                    0.5*P[i*STATE_DIM+j] + 0.5*P[j*STATE_DIM+i],
                                    MIN_COVARIANCE, MAX_COVARIANCE);
                    }
                }

                return Pnew;
            }

            static auto addCovarianceNoise(const matrix & P,
                    const float * noise) -> matrix
            {
                auto Pnew = P;

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    Pnew[k*STATE_DIM+k] = P[k*STATE_DIM+k] + noise[k]*noise[k];
                }

                return Pnew;
            }

            static auto get_pval(const int i, const int j,
                    const float pval, const float minval,
                    const float maxval) -> float
            {
                return
                    isnan(pval) || pval > maxval ? maxval :
                    i==j && pval < minval ? minval :
                    pval;
            }

            // C = x * y
            static auto outer(const vector & x, const vector & y) -> matrix
            {
                auto C = matrix();

                for (size_t i=0; i<STATE_DIM; i++) {
                    for (size_t j=0; j<STATE_DIM; j++) {
                        C[i*STATE_DIM+j] = x[i] * y[j];
                    }
                }

                return C;
            }

            // At = A^T
            static auto trans(const matrix & A) -> matrix
            {
                auto At = matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        At[i*STATE_DIM+j] = A[j*STATE_DIM+i];
                    }
                }

                return At;
            }

            // C = A * B
            static auto dot(const matrix & A, const matrix & B) -> matrix
            {
                auto C = matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        C[i*STATE_DIM+j] = 0;
                        for (int k=0; k<STATE_DIM; ++k) {
                            C[i*STATE_DIM+j] += A[i*STATE_DIM+k] * B[k*STATE_DIM+j];
                        }
                    }
                }

                return C;
            }

            // y = A * x
            static auto dot(const matrix & A, const vector & x) -> vector
            {
                vector y = vector();

                for (int i=0; i<STATE_DIM; i++) {
                    y[i] = 0; 
                    for (int j=0; j<STATE_DIM; j++) {
                        y[i] += A[i*STATE_DIM+j] * x[j];
                    }
                }

                return y;
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
