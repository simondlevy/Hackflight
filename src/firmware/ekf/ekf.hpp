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

            typedef std::array<float, STATE_DIM*STATE_DIM> Matrix;

            typedef std::array<float, STATE_DIM> Vector;

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
                const float pinit[STATE_DIM] = {

                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                core.p = addCovarianceNoise(Matrix(), pinit);

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
                    ThreeAxisSubSampler::finalize(ekf.accel_subsampler_);

                const auto gyro_subsampler =
                    ThreeAxisSubSampler::finalize(ekf.gyro_subsampler_);

                const auto dt = (msec_curr - ekf.last_prediction_msec_) / 1000.f;

                const auto accel = accel_subsampler.sub_sample;
                const auto gyro = gyro_subsampler.sub_sample;

                // The linearized Jacobean matrix
                const auto F = makeJacobian(dt, gyro, ekf.core.x, ekf.r_);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
                const auto P = dot(dot(F, ekf.core.p), trans(F));

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
                auto x = Vector();
                x[STATE_Z] = ekf.core.x[STATE_Z] + ekf.r_.zx * dx + ekf.r_.zy * dy +
                    ekf.r_.zz * dz - GRAVITY * dt2 / 2;

                x[STATE_VX] = ekf.core.x[STATE_VX] + dt * (accelx + gyro.z * tmpSPY -
                        gyro.y * tmpSPZ - GRAVITY * ekf.r_.zx);

                x[STATE_VY] = ekf.core.x[STATE_VY] + dt * (accely - gyro.z * tmpSPX +
                        gyro.x * tmpSPZ - GRAVITY * ekf.r_.zy);

                x[STATE_VZ] = ekf.core.x[STATE_VZ] + dt * (accel.z + gyro.y * tmpSPX -
                        gyro.x * tmpSPY - GRAVITY * ekf.r_.zz);

                x[STATE_D0] = ekf.core.x[STATE_D0];
                x[STATE_D1] = ekf.core.x[STATE_D1];
                x[STATE_D2] = ekf.core.x[STATE_D2];

                // Attitude update (rotate by gyroscope): we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const auto dtw = gyro * dt;

                // compute the quaternion values in [w,x,y,z] order
                auto tmpq = rotate(dtw, ekf.q_);

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

            static auto update(
                    const EKF & ekf,
                    const ImuFilter::Data & imudata,
                    const uint32_t msec_curr) -> EKF
            {
                const auto dt =
                    (msec_curr - ekf.last_process_noise_update_msec_) / 1000.0f;

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

                const auto last_process_noise_update_msec_ =
                    dtpositive ? msec_curr : ekf.last_process_noise_update_msec_;

                const auto accel_subsampler = ThreeAxisSubSampler::accumulate(
                        ekf.accel_subsampler_, imudata.accel_gs);

                const auto gyro_subsampler = ThreeAxisSubSampler::accumulate(
                        ekf.gyro_subsampler_, imudata.gyro_dps);

                const auto gyro_latest = imudata.gyro_dps;

                const auto rzz = ekf.r_.zz;

                const auto rangeok = fabs(rzz) > 0.1 && rzz > 0; 

                const auto coreWithNoise = dtpositive ?
                    Core(ekf.core.x, 
                            enforceSymmetry(addCovarianceNoise(ekf.core.p, noise))) :
                    ekf.core;

                const auto coreWithRange = rangeok && ekf.did_update_with_flow_deck_ ?
                    updateWithRange(
                            coreWithNoise, ekf.zranger_filter_latest_, rzz) :
                    coreWithNoise;

                const auto core_with_range_and_flow = ekf.did_update_with_flow_deck_ ?  
                    updateWithFlow(coreWithRange, ekf.optical_flow_filter_latest_,
                            gyro_latest, rzz):
                    coreWithRange;

                const auto ready = ekf.did_update_with_flow_deck_ || ekf.did_predict_;

                // Incorporate the attitude error (Kalman filter state) with the attitude
                const auto v = ThreeAxis(
                        ekf.core.x[STATE_D0], ekf.core.x[STATE_D1], ekf.core.x[STATE_D2]);

                // reset the attitude error
                const auto x = Vector{
                    core_with_range_and_flow.x[0],
                    core_with_range_and_flow.x[1],
                    core_with_range_and_flow.x[2],
                    core_with_range_and_flow.x[3],
                    0, 0, 0};

                const auto p = enforceSymmetry(core_with_range_and_flow.p);

                const auto q = ready &&
                    (bigenough(v.x) || bigenough(v.y) || bigenough(v.z)) &&
                    smallenough(v.x) && smallenough(v.y) && smallenough(v.z) ?
                    ekf.q_ / Quaternion::l2norm(rotate(v, ekf.q_)) : ekf.q_;

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and accel
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

            static auto update(
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
                    ekf.r_.xx*x[STATE_VX] +
                    ekf.r_.xy*x[STATE_VY] +
                    ekf.r_.xz*x[STATE_VZ];

                // make right positive
                const auto dy = -(
                        ekf.r_.yx*x[STATE_VX] +
                        ekf.r_.yy*x[STATE_VY] +
                        ekf.r_.yz*x[STATE_VZ]); 

                const auto z = x[STATE_Z];

                const auto dz =
                    ekf.r_.zx*x[STATE_VX] +
                    ekf.r_.zy*x[STATE_VY] +
                    ekf.r_.zz*x[STATE_VZ];

                const auto q0 = ekf.q_.w;
                const auto q1 = ekf.q_.x;
                const auto q2 = ekf.q_.y;
                const auto q3 = ekf.q_.z;

                const auto phi = Num::RAD2DEG * atan2f(2*(q2*q3+q0* q1) ,
                        q0*q0 - q1*q1 - q2*q2 + q3*q3);

                const auto dphi = ekf.gyro_latest_.x;

                const auto theta = Num::RAD2DEG * asinf(-2*(q1*q3 - q0*q2));

                const auto dtheta = ekf.gyro_latest_.y;

                const auto psi = Num::RAD2DEG * atan2f(2*(q1*q2+q0* q3),
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

            ThreeAxisSubSampler accel_subsampler_ = ThreeAxisSubSampler(GRAVITY);
            ThreeAxisSubSampler gyro_subsampler_ = ThreeAxisSubSampler(Num::DEG2RAD);

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

                const auto vx = x[STATE_VX];
                const auto vy = x[STATE_VY];
                const auto vz = x[STATE_VZ];

                const auto N = STATE_DIM;

                auto f = Matrix();

                // position
                f[STATE_Z*N+STATE_Z] = 1;
                f[STATE_Z*N+STATE_VX] = r.zx*dt;
                f[STATE_Z*N+STATE_VY] = r.zy*dt;
                f[STATE_Z*N+STATE_VZ] = r.zz*dt;
                f[STATE_Z*N+STATE_D0] = (vy*r.zz - vz*r.zy)*dt;
                f[STATE_Z*N+STATE_D1] = (-vx*r.zz + vz*r.zx)*dt;
                f[STATE_Z*N+STATE_D2] = (vx*r.zy - vy*r.zx)*dt;

                f[STATE_VX*N+STATE_Z] = 0; 
                f[STATE_VX*N+STATE_VX] = 1; 
                f[STATE_VX*N+STATE_VY] = gyro.z*dt;
                f[STATE_VX*N+STATE_VZ] =-gyro.y*dt;
                f[STATE_VX*N+STATE_D0] =  0;
                f[STATE_VX*N+STATE_D1] =  GRAVITY*r.zz*dt;
                f[STATE_VX*N+STATE_D2] = -GRAVITY*r.zy*dt;

                f[STATE_VY*N+STATE_Z] = 0; 
                f[STATE_VY*N+STATE_VX] =-gyro.z*dt;
                f[STATE_VY*N+STATE_VY] = 1; 
                f[STATE_VY*N+STATE_VZ] = gyro.x*dt;
                f[STATE_VY*N+STATE_D0] = -GRAVITY*r.zz*dt;
                f[STATE_VY*N+STATE_D1] =  0;
                f[STATE_VY*N+STATE_D2] =  GRAVITY*r.zx*dt;

                f[STATE_VZ*N+STATE_Z] = 0; 
                f[STATE_VZ*N+STATE_VX] = gyro.y*dt;
                f[STATE_VZ*N+STATE_VY] =-gyro.x*dt;
                f[STATE_VZ*N+STATE_VZ] = 1; 
                f[STATE_VZ*N+STATE_D0] =  GRAVITY*r.zy*dt;
                f[STATE_VZ*N+STATE_D1] = -GRAVITY*r.zx*dt;
                f[STATE_VZ*N+STATE_D2] =  0;

                f[STATE_D0*N+STATE_Z] = 0; 
                f[STATE_D0*N+STATE_VX] = 0; 
                f[STATE_D0*N+STATE_VX] = 0; 
                f[STATE_D0*N+STATE_VZ] = 0; 
                f[STATE_D0*N+STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                f[STATE_D0*N+STATE_D1] =  d2 + d0*d1/2;
                f[STATE_D0*N+STATE_D2] = -d1 + d0*d2/2;

                f[STATE_D1*N+STATE_Z] = 0; 
                f[STATE_D1*N+STATE_VX] = 0; 
                f[STATE_D1*N+STATE_VX] = 0; 
                f[STATE_D1*N+STATE_VZ] = 0; 
                f[STATE_D1*N+STATE_D0] = -d2 + d0*d1/2;
                f[STATE_D1*N+STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                f[STATE_D1*N+STATE_D2] =  d0 + d1*d2/2;

                f[STATE_D2*N+STATE_Z] = 0; 
                f[STATE_D2*N+STATE_VX] = 0; 
                f[STATE_D2*N+STATE_VX] = 0; 
                f[STATE_D2*N+STATE_VZ] = 0; 
                f[STATE_D2*N+STATE_D0] =  d1 + d0*d2/2;
                f[STATE_D2*N+STATE_D1] = -d0 + d1*d2/2;
                f[STATE_D2*N+STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

                return f;
            }

            static auto updateWithFlow(
                    const Core & core,
                    const OpticalFlowFilter & offilter,
                    const ThreeAxis & gyro, const float r22) -> Core
            {
                const auto newcore = updateWithFlowAxis(core, offilter.dt, r22,
                        offilter.dpixelx, offilter.std_dev_x, STATE_VX, gyro.y);

                return updateWithFlowAxis(newcore, offilter.dt, r22,
                        offilter.dpixely, offilter.std_dev_y, STATE_VY, gyro.x);
            }

            static auto updateWithRange(
                    const Core & core,
                    const ZRangerFilter & zrfilter,
                    const float rzz) -> Core
            {

                const auto angle = max(0, fabsf(acosf(rzz)) -
                        Num::DEG2RAD * (15.0f / 2));
                const auto predicted_distance = core.x[STATE_Z] / cosf(angle);
                const auto measured_distance = zrfilter.distance_m;

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                const Vector h = {1 / cosf(angle), 0, 0, 0, 0, 0, 0 };

                return updateWithScalar(core, h,
                        measured_distance-predicted_distance,
                        zrfilter.stdev);
            }

            static auto updateWithFlowAxis(
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

                //float thetapix = Num::DEG2RAD * 4.0f;
                // [rad]    (same in x and y)
                // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
                // corresponding ground length
                const float thetapix = 0.71674f;

                // Saturate elevation in prediction and correction to avoid
                // singularities
                const auto z_g  = max(core.x[STATE_Z], 0.1);

                const auto dg = core.x[state_index];

                const auto omegab = gyroval * Num::DEG2RAD;

                Vector h = {0, 0, 0, 0, 0, 0, 0};

                const auto predicted_n = (dt * Npix / thetapix ) * 
                    ((dg * r22 / z_g) - omegab);

                const auto measured_n = dpixel*FLOW_RESOLUTION;

                h[STATE_Z] = (Npix * dt / thetapix) * 
                    ((r22 * dg) / (-z_g * z_g));

                h[state_index] = (Npix * dt / thetapix) * (r22 / z_g);

                return updateWithScalar(core, h, measured_n-predicted_n,
                        stdev*FLOW_RESOLUTION);
            }

            static auto updateWithScalar(
                    const Core & core,
                    const Vector & h,
                    const float error,
                    const float std_meas_noise) -> Core
            {
                const auto r = std_meas_noise*std_meas_noise;

                const auto pht = dot(core.p, h); // PH'

                float hphr = r; // HPH' + R
                for (size_t i=0; i<STATE_DIM; i++) { 
                    hphr += h[i] * pht[i]; 
                }

                Vector g;
                for (size_t i=0; i<STATE_DIM; i++) {
                    g[i] = pht[i]/hphr; // kalman gain = (PH' (HPH' + R )^-1)
                }

                auto gh = outer(g, h);

                // GH - I
                for (size_t i=0; i<STATE_DIM; i++) { 
                    gh[i*STATE_DIM+i] -= 1; 
                }

                // (GH - I)'
                const auto gh_i = trans(gh);

                // (GH - I)*P
                const auto gh_i_p = dot(gh, core.p); 

                // (GH - I)*P*(GH - I)'
                auto p = dot(gh_i_p, gh_i);

                // State update
                auto x = Vector();
                for (int i=0; i<STATE_DIM; i++) {
                    x[i] = core.x[i] + g[i] * error; 
                }

                // Add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        const auto v = g[i] * r * g[j];

                        // add measurement noise
                        p[i*STATE_DIM+j] = p[j*STATE_DIM+i] =
                            get_pval(i, j, 0.5*p[i*STATE_DIM+j] + 0.5*p[j*STATE_DIM+i] + v,
                                    MIN_COVARIANCE, MAX_COVARIANCE); 
                    }
                }

                return Core(x, p);
            }

            static auto enforceSymmetry(const Matrix & P) -> Matrix
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

            static auto addCovarianceNoise(const Matrix & P,
                    const float * noise) -> Matrix
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
            static auto outer(const Vector & x, const Vector & y) -> Matrix
            {
                auto C = Matrix();

                for (size_t i=0; i<STATE_DIM; i++) {
                    for (size_t j=0; j<STATE_DIM; j++) {
                        C[i*STATE_DIM+j] = x[i] * y[j];
                    }
                }

                return C;
            }

            // At = A^T
            static auto trans(const Matrix & a) -> Matrix
            {
                auto at = Matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        at[i*STATE_DIM+j] = a[j*STATE_DIM+i];
                    }
                }

                return at;
            }

            // C = A * B
            static auto dot(const Matrix & a, const Matrix & b) -> Matrix
            {
                auto c = Matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        c[i*STATE_DIM+j] = 0;
                        for (int k=0; k<STATE_DIM; ++k) {
                            c[i*STATE_DIM+j] += a[i*STATE_DIM+k] * b[k*STATE_DIM+j];
                        }
                    }
                }

                return c;
            }

            // y = A * x
            static auto dot(const Matrix & a, const Vector & x) -> Vector
            {
                auto y = Vector();

                for (int i=0; i<STATE_DIM; i++) {
                    y[i] = 0; 
                    for (int j=0; j<STATE_DIM; j++) {
                        y[i] += a[i*STATE_DIM+j] * x[j];
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
