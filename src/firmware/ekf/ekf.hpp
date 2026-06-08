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
#include <firmware/opticalflow/filter.hpp>
#include <firmware/zranger/filter.hpp>
#include <num.hpp>

namespace hf {

    class EKF { 

        private:

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float kStdevInitialPositionZ = 1;
            static constexpr float kStdevInitialVelocity = 0.01;
            static constexpr float kStdevInitialAttitudeRollPitch = 0.01;
            static constexpr float kStdevInitialAttitudeYaw = 0.01;

            static constexpr float kProcessNoiseAccelXy = 0.5f;
            static constexpr float kProcessNoiseAccelZ = 1.0f;
            static constexpr float kProcessNoiseVelocity = 0;
            static constexpr float kProcessNoisePosition = 0;
            static constexpr float kProcessNoiseAttitude = 0;
            static constexpr float kMeasurementNoiseGyroRollPitch = 0.1f; // radians per second
            static constexpr float kMeasurementNoiseGyroYaw = 0.1f;       // radians per second

            static constexpr float kGravity = 9.81;

            //We do get the measurements in 10x the motion pixels (experimentally measured)
            static constexpr float kFlowResolution = 0.1;

            // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
            static constexpr float MaxCovariance = 100;
            static constexpr float MinCovariance = 1e-6;

            // the reversion of pitch and roll to zero
            static constexpr float kRollPitchZeroReversion = 0.001;

            static constexpr float MinAngle = 1e-4;
            static constexpr float MaxAngle = 10;

            // Indexes to acceless the vehicle's state, stored as a column vector
            enum
            {
                kStateZ,
                kStateVx,
                kStateVy,
                kStateVz,
                kStateD0,
                kStateD1,
                kStateD2,
                kStateDim
            };

            typedef std::array<float, kStateDim*kStateDim> Matrix;

            typedef std::array<float, kStateDim> Vector;

            class Core {

                public:

                    // State vector
                    Vector x;

                    // Covariance matrix
                    Matrix p;

                    Core() = default;

                    Core& operator=(const Core& other) = default;

                    Core(const Vector & x, const Matrix & p) 
                        : x(x), p(p) {}
            };

            Core core;

        public:

            EKF& operator=(const EKF& other) = default;

            EKF()
            {
                // Add in the initial process noise 
                const float pinit[kStateDim] = {

                    kStdevInitialPositionZ,
                    kStdevInitialVelocity,
                    kStdevInitialVelocity,
                    kStdevInitialVelocity,
                    kStdevInitialAttitudeRollPitch,
                    kStdevInitialAttitudeRollPitch,
                    kStdevInitialAttitudeYaw
                };

                core.p = AddCovarianceNoise(Matrix(), pinit);

                did_update_with_flow_deck_ = false;
                last_process_noise_update_msec_ = 0;

                did_predict_ = false;
                last_prediction_msec_ = 0;
            }

            EKF(
                    const Core & core,
                    const Quaternion & q,
                    const Rotation & r,
                    const ThreeAxis & gyro_latest,
                    const ThreeAxisSubSampler & accel_subsampler,
                    const ThreeAxisSubSampler & gyro_subsampler,
                    const bool did_update_with_flow_deck,
                    const ZRangerFilter & zranger_filter_latest_,
                    const OpticalFlowFilter & optical_flow_filter_latest_,
                    const uint32_t last_process_noise_update_msec_,
                    const bool did_predict,
                    const uint32_t last_prediction_msec)
                :
                    core(core),
                    q_(q),
                    r_(r),
                    gyro_latest_(gyro_latest),
                    accel_subsampler_(accel_subsampler),
                    gyro_subsampler_(gyro_subsampler),
                    did_update_with_flow_deck_(did_update_with_flow_deck),
                    zranger_filter_latest_(zranger_filter_latest_),
                    optical_flow_filter_latest_(optical_flow_filter_latest_),
                    last_process_noise_update_msec_(last_process_noise_update_msec_),
                    did_predict_(did_predict),
                    last_prediction_msec_(last_prediction_msec) {}

            static auto Predict(const EKF & ekf, const uint32_t msec_curr,
                    const bool isFlying) -> EKF
            {
                const auto accel_subsampler =
                    ThreeAxisSubSampler::Finalize(ekf.accel_subsampler_);

                const auto gyro_subsampler =
                    ThreeAxisSubSampler::Finalize(ekf.gyro_subsampler_);

                const auto dt = (msec_curr - ekf.last_prediction_msec_) / 1000.f;

                const auto accel = accel_subsampler.sub_sample;
                const auto gyro = gyro_subsampler.sub_sample;

                // The linearized Jacobean matrix
                const auto F = makeJacobian(dt, gyro, ekf.core.x, ekf.r_);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
                const auto P = Dot(Dot(F, ekf.core.p), Transpose(F));

                const auto dt2 = dt * dt;

                // keep previous time step's state for the update
                const auto tmpSPX = ekf.core.x[kStateVx];
                const auto tmpSPY = ekf.core.x[kStateVy];
                const auto tmpSPZ = ekf.core.x[kStateVz];

                // position updates in the body frame (will be Rotated to inertial frame)
                const auto dx = ekf.core.x[kStateVx] * dt + (isFlying ? 0 : accel.x * dt2 / 2);
                const auto dy = ekf.core.x[kStateVy] * dt + (isFlying ? 0 : accel.y * dt2 / 2);

                // thrust can only be produced in the body's Z direction
                const auto dz = ekf.core.x[kStateVz] * dt + accel.z * dt2 / 2; 

                const auto accelx = isFlying ? 0 : accel.x;
                const auto accely = isFlying ? 0 : accel.y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                auto x = Vector();
                x[kStateZ] = ekf.core.x[kStateZ] + ekf.r_.zx * dx + ekf.r_.zy * dy +
                    ekf.r_.zz * dz - kGravity * dt2 / 2;

                x[kStateVx] = ekf.core.x[kStateVx] + dt * (accelx + gyro.z * tmpSPY -
                        gyro.y * tmpSPZ - kGravity * ekf.r_.zx);

                x[kStateVy] = ekf.core.x[kStateVy] + dt * (accely - gyro.z * tmpSPX +
                        gyro.x * tmpSPZ - kGravity * ekf.r_.zy);

                x[kStateVz] = ekf.core.x[kStateVz] + dt * (accel.z + gyro.y * tmpSPX -
                        gyro.x * tmpSPY - kGravity * ekf.r_.zz);

                x[kStateD0] = ekf.core.x[kStateD0];
                x[kStateD1] = ekf.core.x[kStateD1];
                x[kStateD2] = ekf.core.x[kStateD2];

                // Attitude update (Rotate by gyroscope): we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const auto dtw = gyro * dt;

                // compute the quaternion values in [w,x,y,z] order
                auto tmpq = Rotate(dtw, ekf.q_);

                const auto keep = 1.0f - kRollPitchZeroReversion;

                const auto newtmpq = isFlying ? tmpq : 
                    Quaternion(
                            tmpq.w = keep * tmpq.w + kRollPitchZeroReversion,
                            tmpq.x = keep * tmpq.x, 
                            tmpq.y = keep * tmpq.y, 
                            tmpq.z = keep * tmpq.z); 

                // normalize and store the result
                const auto q = newtmpq / Quaternion::L2Norm(newtmpq);

                return EKF(
                        Core(x, P),
                        q,
                        ekf.r_,
                        ekf.gyro_latest_,
                        accel_subsampler,
                        gyro_subsampler,
                        ekf.did_update_with_flow_deck_,
                        ekf.zranger_filter_latest_,
                        ekf.optical_flow_filter_latest_,
                        ekf.last_process_noise_update_msec_,
                        true, // did_predict,
                        msec_curr);  // last_prediction_msec

            } // predict

            static auto Update(
                    const EKF & ekf,
                    const ImuFilter::Data & imudata,
                    const uint32_t msec_curr) -> EKF
            {
                const auto dt =
                    (msec_curr - ekf.last_process_noise_update_msec_) / 1000.0f;

                const auto dtpositive = dt > 0;

                const float noise[kStateDim] = {
                    kProcessNoiseAccelZ*dt*dt + kProcessNoiseVelocity*dt +
                        kProcessNoisePosition,
                    kProcessNoiseAccelXy*dt + kProcessNoiseVelocity,
                    kProcessNoiseAccelXy*dt + kProcessNoiseVelocity,
                    kProcessNoiseAccelZ*dt + kProcessNoiseVelocity,
                    kMeasurementNoiseGyroRollPitch * dt + kProcessNoiseAttitude,
                    kMeasurementNoiseGyroRollPitch * dt + kProcessNoiseAttitude,
                    kMeasurementNoiseGyroYaw * dt + kProcessNoiseAttitude
                };

                const auto last_process_noise_update_msec_ =
                    dtpositive ? msec_curr : ekf.last_process_noise_update_msec_;

                const auto accel_subsampler = ThreeAxisSubSampler::Accumulate(
                        ekf.accel_subsampler_, imudata.accel_gs);

                const auto gyro_subsampler = ThreeAxisSubSampler::Accumulate(
                        ekf.gyro_subsampler_, imudata.gyro_dps);

                const auto gyro_latest = imudata.gyro_dps;

                const auto rzz = ekf.r_.zz;

                const auto rangeok = fabs(rzz) > 0.1 && rzz > 0; 

                const auto coreWithNoise = dtpositive ?
                    Core(ekf.core.x, 
                            EnforceSymmetry(AddCovarianceNoise(ekf.core.p, noise))) :
                    ekf.core;

                const auto coreWithRange = rangeok && ekf.did_update_with_flow_deck_ ?
                    UpdateWithRange(
                            coreWithNoise, ekf.zranger_filter_latest_, rzz) :
                    coreWithNoise;

                const auto core_with_range_and_flow = ekf.did_update_with_flow_deck_ ?  
                    UpdateWithFlow(coreWithRange, ekf.optical_flow_filter_latest_,
                            gyro_latest, rzz):
                    coreWithRange;

                const auto ready = ekf.did_update_with_flow_deck_ || ekf.did_predict_;

                // Incorporate the attitude error (Kalman filter state) with the attitude
                const auto v = ThreeAxis(
                        ekf.core.x[kStateD0], ekf.core.x[kStateD1], ekf.core.x[kStateD2]);

                // reset the attitude error
                const auto x = Vector{
                    core_with_range_and_flow.x[0],
                    core_with_range_and_flow.x[1],
                    core_with_range_and_flow.x[2],
                    core_with_range_and_flow.x[3],
                    0, 0, 0};

                const auto p = EnforceSymmetry(core_with_range_and_flow.p);

                const auto q = ready &&
                    (IsBigEnough(v.x) || IsBigEnough(v.y) || IsBigEnough(v.z)) &&
                    IsSmallEnough(v.x) && IsSmallEnough(v.y) && IsSmallEnough(v.z) ?
                    ekf.q_ / Quaternion::L2Norm(Rotate(v, ekf.q_)) : ekf.q_;

                // Convert the new attitude to a rotation matrix, such that we can
                // Rotate body-frame velocity and accel
                const auto r = ready ?

                    Rotation(
                            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
                            2 * q.x * q.y - 2 * q.w * q.z,
                            2 * q.x * q.z + 2 * q.w * q.y,
                            2 * q.x * q.y + 2 * q.w * q.z,
                            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
                            2 * q.y * q.z - 2 * q.w * q.x,
                            2 * q.x * q.z - 2 * q.w * q.y,
                            2 * q.y * q.z + 2 * q.w * q.x,
                            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) :
                        
                        ekf.r_;

                const auto core = ready ? Core(x, p) : core_with_range_and_flow;

                return EKF(
                        core,
                        q,
                        r,
                        gyro_latest,
                        accel_subsampler,
                        gyro_subsampler,
                        false, // didUpateWithFlowDeck
                        ekf.zranger_filter_latest_,
                        ekf.optical_flow_filter_latest_,
                        last_process_noise_update_msec_,
                        false, // did_predict
                        ekf.last_prediction_msec_);

            } // update

            static auto Update(
                    const EKF &ekf,
                    const ZRangerFilter & zrfilter,
                    const OpticalFlowFilter & offilter) -> EKF
            {
                return EKF(
                        ekf.core,
                        ekf.q_,
                        ekf.r_,
                        ekf.gyro_latest_,
                        ekf.accel_subsampler_,
                        ekf.gyro_subsampler_,
                        true, // did_update_with_flow_deck,
                        zrfilter,
                        offilter,
                        ekf.last_process_noise_update_msec_,
                        ekf. did_predict_,
                        ekf. last_prediction_msec_);
            }

            static auto getVehicleState(const EKF & ekf) -> VehicleState
            {
                const auto x = ekf.core.x;

                const auto dx =
                    ekf.r_.xx*x[kStateVx] +
                    ekf.r_.xy*x[kStateVy] +
                    ekf.r_.xz*x[kStateVz];

                // make right positive
                const auto dy = -(
                        ekf.r_.yx*x[kStateVx] +
                        ekf.r_.yy*x[kStateVy] +
                        ekf.r_.yz*x[kStateVz]); 

                const auto z = x[kStateZ];

                const auto dz =
                    ekf.r_.zx*x[kStateVx] +
                    ekf.r_.zy*x[kStateVy] +
                    ekf.r_.zz*x[kStateVz];

                const auto q0 = ekf.q_.w;
                const auto q1 = ekf.q_.x;
                const auto q2 = ekf.q_.y;
                const auto q3 = ekf.q_.z;

                const auto phi = Num::kRad2Deg * atan2f(2*(q2*q3+q0* q1) ,
                        q0*q0 - q1*q1 - q2*q2 + q3*q3);

                const auto dphi = ekf.gyro_latest_.x;

                const auto theta = Num::kRad2Deg * asinf(-2*(q1*q3 - q0*q2));

                const auto dtheta = ekf.gyro_latest_.y;

                const auto psi = Num::kRad2Deg * atan2f(2*(q1*q2+q0* q3),
                        q0*q0 + q1*q1 - q2*q2 - q3*q3); 

                const auto dpsi = ekf.gyro_latest_.z;

                // Return psi/dpsi nose-right positive
                return VehicleState(
                        dx, dy, z, dz, phi, dphi, theta, dtheta, -psi, -dpsi);
            }

        private:

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Quaternion q_;

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Rotation r_;

            ThreeAxis gyro_latest_;

            ThreeAxisSubSampler accel_subsampler_ = ThreeAxisSubSampler(kGravity);
            ThreeAxisSubSampler gyro_subsampler_ = ThreeAxisSubSampler(Num::kDeg2Rad);

            bool did_update_with_flow_deck_;

            ZRangerFilter zranger_filter_latest_;
            OpticalFlowFilter optical_flow_filter_latest_;

            uint32_t last_process_noise_update_msec_;

            bool did_predict_;
            uint32_t last_prediction_msec_;

            static auto makeJacobian(
                    const float dt,
                    const ThreeAxis & gyro,
                    const Vector x,
                    const Rotation & r) -> Matrix
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x[kStateVx];
                const auto vy = x[kStateVy];
                const auto vz = x[kStateVz];

                const auto N = kStateDim;

                auto f = Matrix();

                // position
                f[kStateZ*N+kStateZ] = 1;
                f[kStateZ*N+kStateVx] = r.zx*dt;
                f[kStateZ*N+kStateVy] = r.zy*dt;
                f[kStateZ*N+kStateVz] = r.zz*dt;
                f[kStateZ*N+kStateD0] = (vy*r.zz - vz*r.zy)*dt;
                f[kStateZ*N+kStateD1] = (-vx*r.zz + vz*r.zx)*dt;
                f[kStateZ*N+kStateD2] = (vx*r.zy - vy*r.zx)*dt;

                f[kStateVx*N+kStateZ] = 0; 
                f[kStateVx*N+kStateVx] = 1; 
                f[kStateVx*N+kStateVy] = gyro.z*dt;
                f[kStateVx*N+kStateVz] =-gyro.y*dt;
                f[kStateVx*N+kStateD0] =  0;
                f[kStateVx*N+kStateD1] =  kGravity*r.zz*dt;
                f[kStateVx*N+kStateD2] = -kGravity*r.zy*dt;

                f[kStateVy*N+kStateZ] = 0; 
                f[kStateVy*N+kStateVx] =-gyro.z*dt;
                f[kStateVy*N+kStateVy] = 1; 
                f[kStateVy*N+kStateVz] = gyro.x*dt;
                f[kStateVy*N+kStateD0] = -kGravity*r.zz*dt;
                f[kStateVy*N+kStateD1] =  0;
                f[kStateVy*N+kStateD2] =  kGravity*r.zx*dt;

                f[kStateVz*N+kStateZ] = 0; 
                f[kStateVz*N+kStateVx] = gyro.y*dt;
                f[kStateVz*N+kStateVy] =-gyro.x*dt;
                f[kStateVz*N+kStateVz] = 1; 
                f[kStateVz*N+kStateD0] =  kGravity*r.zy*dt;
                f[kStateVz*N+kStateD1] = -kGravity*r.zx*dt;
                f[kStateVz*N+kStateD2] =  0;

                f[kStateD0*N+kStateZ] = 0; 
                f[kStateD0*N+kStateVx] = 0; 
                f[kStateD0*N+kStateVx] = 0; 
                f[kStateD0*N+kStateVz] = 0; 
                f[kStateD0*N+kStateD0] =  1 - d1*d1/2 - d2*d2/2;
                f[kStateD0*N+kStateD1] =  d2 + d0*d1/2;
                f[kStateD0*N+kStateD2] = -d1 + d0*d2/2;

                f[kStateD1*N+kStateZ] = 0; 
                f[kStateD1*N+kStateVx] = 0; 
                f[kStateD1*N+kStateVx] = 0; 
                f[kStateD1*N+kStateVz] = 0; 
                f[kStateD1*N+kStateD0] = -d2 + d0*d1/2;
                f[kStateD1*N+kStateD1] =  1 - d0*d0/2 - d2*d2/2;
                f[kStateD1*N+kStateD2] =  d0 + d1*d2/2;

                f[kStateD2*N+kStateZ] = 0; 
                f[kStateD2*N+kStateVx] = 0; 
                f[kStateD2*N+kStateVx] = 0; 
                f[kStateD2*N+kStateVz] = 0; 
                f[kStateD2*N+kStateD0] =  d1 + d0*d2/2;
                f[kStateD2*N+kStateD1] = -d0 + d1*d2/2;
                f[kStateD2*N+kStateD2] = 1 - d0*d0/2 - d1*d1/2;

                return f;
            }

            static auto UpdateWithFlow(
                    const Core & core,
                    const OpticalFlowFilter & offilter,
                    const ThreeAxis & gyro, const float r22) -> Core
            {
                const auto newcore = UpdateWithFlowAxis(core, offilter.dt, r22,
                        offilter.dpixelx, offilter.std_dev_x, kStateVx, gyro.y);

                return UpdateWithFlowAxis(newcore, offilter.dt, r22,
                        offilter.dpixely, offilter.std_dev_y, kStateVy, gyro.x);
            }

            static auto UpdateWithRange(
                    const Core & core,
                    const ZRangerFilter & zrfilter,
                    const float rzz) -> Core
            {

                const auto angle = max(0, fabsf(acosf(rzz)) -
                        Num::kDeg2Rad * (15.0f / 2));
                const auto predicted_distance = core.x[kStateZ] / cosf(angle);
                const auto measured_distance = zrfilter.distance_m;

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                const Vector h = {1 / cosf(angle), 0, 0, 0, 0, 0, 0 };

                return UpdateWithScalar(core, h,
                        measured_distance-predicted_distance,
                        zrfilter.stdev);
            }

            static auto UpdateWithFlowAxis(
                    const Core & core,
                    const float dt,
                    const float r22,
                    const float dpixel,
                    const float stdev,
                    const uint8_t state_index,
                    const float gyroval) -> Core
            {
                // [pixels] (same in x and y)
                const float Npix = 35.0;                      

                //float thetapix = Num::kDeg2Rad * 4.0f;
                // [rad]    (same in x and y)
                // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
                // corresponding ground length
                const float thetapix = 0.71674f;

                // Saturate elevation in prediction and correction to avoid
                // singularities
                const auto z_g  = max(core.x[kStateZ], 0.1);

                const auto dg = core.x[state_index];

                const auto omegab = gyroval * Num::kDeg2Rad;

                Vector h = {0, 0, 0, 0, 0, 0, 0};

                const auto predicted_n = (dt * Npix / thetapix ) * 
                    ((dg * r22 / z_g) - omegab);

                const auto measured_n = dpixel*kFlowResolution;

                h[kStateZ] = (Npix * dt / thetapix) * 
                    ((r22 * dg) / (-z_g * z_g));

                h[state_index] = (Npix * dt / thetapix) * (r22 / z_g);

                return UpdateWithScalar(core, h, measured_n-predicted_n,
                        stdev*kFlowResolution);
            }

            static auto UpdateWithScalar(
                    const Core & core,
                    const Vector & h,
                    const float error,
                    const float std_meas_noise) -> Core
            {
                const auto r = std_meas_noise*std_meas_noise;

                const auto pht = Dot(core.p, h); // PH'

                float hphr = r; // HPH' + R
                for (size_t i=0; i<kStateDim; i++) { 
                    hphr += h[i] * pht[i]; 
                }

                Vector g;
                for (size_t i=0; i<kStateDim; i++) {
                    g[i] = pht[i]/hphr; // kalman gain = (PH' (HPH' + R )^-1)
                }

                auto gh = Outer(g, h);

                // GH - I
                for (size_t i=0; i<kStateDim; i++) { 
                    gh[i*kStateDim+i] -= 1; 
                }

                // (GH - I)'
                const auto gh_i = Transpose(gh);

                // (GH - I)*P
                const auto gh_i_p = Dot(gh, core.p); 

                // (GH - I)*P*(GH - I)'
                auto p = Dot(gh_i_p, gh_i);

                // State update
                auto x = Vector();
                for (int i=0; i<kStateDim; i++) {
                    x[i] = core.x[i] + g[i] * error; 
                }

                // Add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<kStateDim; i++) {

                    for (int j=i; j<kStateDim; j++) {

                        const auto v = g[i] * r * g[j];

                        // add measurement noise
                        p[i*kStateDim+j] = p[j*kStateDim+i] =
                            GetPval(i, j, 0.5*p[i*kStateDim+j] + 0.5*p[j*kStateDim+i] + v,
                                    MinCovariance, MaxCovariance); 
                    }
                }

                return Core(x, p);
            }

            static auto EnforceSymmetry(const Matrix & P) -> Matrix
            {
                auto Pnew = P;

                for (int i=0; i<kStateDim; i++) {

                    for (int j=i; j<kStateDim; j++) {

                        Pnew[i*kStateDim+j] = Pnew[j*kStateDim+i] =
                            GetPval(i, j,
                                    0.5*P[i*kStateDim+j] + 0.5*P[j*kStateDim+i],
                                    MinCovariance, MaxCovariance);
                    }
                }

                return Pnew;
            }

            static auto AddCovarianceNoise(const Matrix & P,
                    const float * noise) -> Matrix
            {
                auto Pnew = P;

                for (uint8_t k=0; k<kStateDim; ++k) {
                    Pnew[k*kStateDim+k] = P[k*kStateDim+k] + noise[k]*noise[k];
                }

                return Pnew;
            }

            static auto GetPval(const int i, const int j,
                    const float pval, const float minval,
                    const float maxval) -> float
            {
                return
                    isnan(pval) || pval > maxval ? maxval :
                    i==j && pval < minval ? minval :
                    pval;
            }

            static auto IsBigEnough(const float v) -> bool
            {
                return fabsf(v) > MinAngle;
            }

            static auto IsSmallEnough(const float v) -> bool
            {
                return fabsf(v) < MaxAngle;
            }

            // C = x * y
            static auto Outer(const Vector & x, const Vector & y) -> Matrix
            {
                auto C = Matrix();

                for (size_t i=0; i<kStateDim; i++) {
                    for (size_t j=0; j<kStateDim; j++) {
                        C[i*kStateDim+j] = x[i] * y[j];
                    }
                }

                return C;
            }

            // At = A^T
            static auto Transpose(const Matrix & a) -> Matrix
            {
                auto at = Matrix();

                for (int i=0; i<kStateDim; ++i) {
                    for (int j=0; j<kStateDim; ++j) {
                        at[i*kStateDim+j] = a[j*kStateDim+i];
                    }
                }

                return at;
            }

            // C = A * B
            static auto Dot(const Matrix & a, const Matrix & b) -> Matrix
            {
                auto c = Matrix();

                for (int i=0; i<kStateDim; ++i) {
                    for (int j=0; j<kStateDim; ++j) {
                        c[i*kStateDim+j] = 0;
                        for (int k=0; k<kStateDim; ++k) {
                            c[i*kStateDim+j] += a[i*kStateDim+k] * b[k*kStateDim+j];
                        }
                    }
                }

                return c;
            }

            // y = A * x
            static auto Dot(const Matrix & a, const Vector & x) -> Vector
            {
                auto y = Vector();

                for (int i=0; i<kStateDim; i++) {
                    y[i] = 0; 
                    for (int j=0; j<kStateDim; j++) {
                        y[i] += a[i*kStateDim+j] * x[j];
                    }
                }

                return y;
            }

            static auto Rotate(
                    const ThreeAxis & v, const Quaternion & q)-> Quaternion
            {
                const auto angle = ThreeAxis::L2Norm(v);
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
