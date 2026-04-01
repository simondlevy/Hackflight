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

#include <firmware/ekf/datatypes.hpp>
#include <firmware/ekf/imu_subsampler.hpp>
#include <firmware/zranger/filter.hpp>
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
                STATE_Z,   // 0
                STATE_VX,  // 1
                STATE_VY,  // 2
                STATE_VZ,  // 3
                STATE_D0,  // 4
                STATE_D1,  // 5
                STATE_D2,  // 6
                STATE_DIM
            };

        public:

            EKF& operator=(const EKF& other) = default;

            EKF() 
            {
                x = xinit();
                P = pinit();
                q = qinit();
                accelSubSampler = ImuSubSampler();
                gyroSubSampler = ImuSubSampler();
            }

            EKF (
                const Vector & x,
                const Matrix & P,
                const Vector & q,
                const Matrix R,
                const ImuSubSampler  & accelSubSampler,
                const ImuSubSampler  & gyroSubSampler,
                const uint32_t imuSampleCount,
                const bool didResetEstimation,
                const bool isUpdated,
                const uint32_t lastPredictionMs,
                const uint32_t lastProcessNoiseUpdateMs) 
                :
                    x(x),
                    P(P),
                    q(q),
                    R(R),
                    accelSubSampler(accelSubSampler),
                    gyroSubSampler(gyroSubSampler), 
                    imuSampleCount(imuSampleCount),
                    didResetEstimation(didResetEstimation),
                    isUpdated(isUpdated),
                    lastPredictionMs(lastPredictionMs) {}

            static auto predict(const EKF & ekf, const uint32_t msec_curr,
                    const bool isFlying) -> EKF
            {
                const auto accelSubSampler =
                    ImuSubSampler::finalize(
                            ekf.accelSubSampler, ekf.imuSampleCount);

                const auto gyroSubSampler =
                    ImuSubSampler::finalize(
                            ekf.gyroSubSampler, ekf.imuSampleCount);

                const auto dt = (msec_curr - ekf.lastPredictionMs) / 1000.f;

                const auto gyro =
                    ImuSubSampler::getOutput(gyroSubSampler, Num::DEG2RAD);

                const auto accel =
                    ImuSubSampler::getOutput(accelSubSampler,  G);

                const auto F = makeJacobian(ekf.x, ekf.R, gyro, dt);

                // P_k = F_{k-1} P_{k-1} F^T_{k-1}
                const auto P = (F * ekf.P) * F.transpose();

                // keep previous time step's state for the update
                const auto tmpSPX = 0;
                const auto tmpSPY = 0;
                const auto tmpSPZ = ekf.x(STATE_VZ);

                const auto dt2 = dt * dt;

                // position updates in the body frame (will be rotated to inertial frame)
                const auto dx = isFlying ? 0 : accel(0) * dt2 / 2;
                const auto dy = isFlying ? 0 : accel(1) * dt2 / 2;

                // thrust can only be produced in the body's Z direction
                const auto dz = ekf.x(STATE_VZ) * dt + accel(2) * dt2 / 2; 

                // position update
                const auto x0 = ekf.x(STATE_Z) + ekf.R(2,0) * dx + ekf.R(2,1) * dy
                    + ekf.R(2,2) * dz - G * dt2 / 2;

                const auto accelx = isFlying ? 0 : accel(0);
                const auto accely = isFlying ? 0 : accel(1);

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                const auto x1 = ekf.x(STATE_VX) + dt * (accelx + gyro(2) *
                        tmpSPY - gyro(1) * tmpSPZ
                        - G * ekf.R(2,0));
                const auto x2 = ekf.x(STATE_VY) + dt * (accely - gyro(2) *
                        tmpSPX + gyro(0) * tmpSPZ
                        - G * ekf.R(2,1));
                const auto x3 = ekf.x(STATE_VZ) + dt * (accel(2) + gyro(1) *
                        tmpSPX - gyro(0) * tmpSPY
                        - G * ekf.R(2,2));

                // attitude update (rotate by gyro), we do this in quaternions
                // this is the gyro angular velocity integrated over the sample
                // period
                const auto dtwx = dt*gyro(0);
                const auto dtwy = dt*gyro(1);
                const auto dtwz = dt*gyro(2);

                // Rotate the vehicle's attitude by the delta quaternion vector
                const auto pq = rotate(ekf.q, dtwx, dtwy, dtwz);

                // Quaternion used for initial orientation
                Vector qinit = Vector(4);
                qinit << 1, 0, 0, 0;

                const auto keep = 1.0f - ROLLPITCH_ZERO_REVERSION;
                const auto pqnew = isFlying ? pq :
                    keep * pq + ROLLPITCH_ZERO_REVERSION * qinit;

                Vector x = Vector(STATE_DIM);
                x << x0, x1, x2, x3, ekf.x(4), ekf.x(5), ekf.x(6); 

                return EKF(
                        x,
                        P,
                        qnorm(pqnew), // q
                        ekf.R,
                        accelSubSampler,
                        gyroSubSampler,
                        0,            // imuSampleCount
                        ekf.didResetEstimation,
                        true,         // isUpdated
                        msec_curr,    // lastPredictionMs
                        ekf.lastProcessNoiseUpdateMs);

            } // predict

            static auto updateWithImu(const EKF & ekf, const uint32_t msec_curr,
                    const ImuFiltered & imudata) -> EKF
            {
                const auto accelSubSampler = ekf.didResetEstimation ?
                    ImuSubSampler() : ekf.accelSubSampler;

                const auto accelSubSampler_ =
                    ImuSubSampler::accumulate(accelSubSampler,
                            imudata.accelGs);

                const auto imuSampleCount = ekf.imuSampleCount + 1;

                const auto gyroSubSampler = ekf.didResetEstimation ?
                    ImuSubSampler() : ekf.gyroSubSampler;

                const auto gyroSubSampler_ =
                    ImuSubSampler::accumulate(gyroSubSampler,
                            imudata.gyroDps);

                const auto isUpdated = ekf.didResetEstimation ? false : ekf.isUpdated;

                const auto lastPredictionMs = ekf.didResetEstimation ?
                    msec_curr : ekf.lastPredictionMs;

                const auto lastProcessNoiseUpdateMs =
                    ekf.didResetEstimation ?  msec_curr :
                    ekf.lastProcessNoiseUpdateMs;

                const float dt = (msec_curr - lastProcessNoiseUpdateMs)
                    / 1000.0f;

                // XXX can we compress this?
                const auto x = ekf.didResetEstimation ? xinit() : ekf.x;
                const auto x_ = dt > 0 ? enforceSymmetry(x) : x;
                const auto x__ = isUpdated ? enforceSymmetry(x_) : x_;

                // XXX ditto ^^^
                const auto P = ekf.didResetEstimation ? pinit() : ekf.P;
                const auto P_ = dt > 0 ? enforceSymmetry(pnoisy(P, dt)) : P;
                const auto P__ = isUpdated ?
                    enforceSymmetry(pnoisy(P_, dt)) : P_;

                const auto lastProcessNoiseUpdateMs_ = dt > 0 ? msec_curr :
                    lastProcessNoiseUpdateMs;

                const auto q = ekf.didResetEstimation ? qinit() : ekf.q;
                const auto q_ =
                    isUpdated ? tryToToIncorporateAttitude(q, x_) : q;

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and acc
                const auto R = isUpdated ? quat2rotation(q_) : ekf.R;

                const float dx = 0;//R(0,0)*_x(STATE_VX) + R(0,1)*_x(STATE_VY) + R(0,2)*_x(STATE_VZ);
                const float dy = 0;//R(1,0)*_x(STATE_VX) + R(1,1)*_x(STATE_VY) + R(1,2)*_x(STATE_VZ); 
                const float dz = 0;//R(2,0)*ekf.x(STATE_VX) + R(2,1)*ekf.x(STATE_VY) + R(2,2)*ekf.x(STATE_VZ);

                const auto didResetEstimation =
                    !areVelsInBounds(dx, dy, dz) ?
                    true :
                    ekf.didResetEstimation;

                return EKF(
                        x__,
                        P__,
                        q_,
                        R,
                        accelSubSampler_,
                        gyroSubSampler_,
                        imuSampleCount,
                        didResetEstimation,
                        false, // isUpdated
                        lastPredictionMs,
                        lastProcessNoiseUpdateMs_);

            } // update

            static auto getVehicleState(const EKF & ekf,
                    const ImuFiltered & imudata) -> VehicleState
            {
                const auto gyroDps = imudata.gyroDps;

                const auto phi = Num::RAD2DEG *
                    atan2f(2*(ekf.q(2)*ekf.q(3)+ekf.q(0)* ekf.q(1)),
                            ekf.q(0)*ekf.q(0) - ekf.q(1)*ekf.q(1) -
                            ekf.q(2)*ekf.q(2) + ekf.q(3)*ekf.q(3));

                const auto dphi = gyroDps.x;

                const auto theta = Num::RAD2DEG *
                    asinf(-2*(ekf.q(1)*ekf.q(3) - ekf.q(0)*ekf.q(2)));

                const auto dtheta = gyroDps.y;

                const auto psi = Num::RAD2DEG *
                    atan2f(2*(ekf.q(1)*ekf.q(2)+ekf.q(0)* ekf.q(3)),
                            ekf.q(0)*ekf.q(0) + ekf.q(1)*ekf.q(1) -
                            ekf.q(2)*ekf.q(2) - ekf.q(3)*ekf.q(3)); 

                const auto dpsi = gyroDps.z;

                const auto z = ekf.x(STATE_Z);

                const float dx = 0;
                const float dy = 0;
                const float dz = ekf.x(STATE_VZ);

                return VehicleState(dx, -dy, z, dz, phi, dphi, theta, dtheta,
                        -psi, -dpsi); // make nose-right positive
            }

            static auto updateWithZRange(const EKF & ekf,
                    const ZRangerFilter & zrfilter) -> EKF
            {
                const auto r22 = ekf.R(2,2);

                const auto angle =
                    fmax(STATE_Z, fabsf(acosf(r22)) - Num::DEG2RAD * (15.0f / 2.0f));

                const auto predicted_distance = ekf.x(STATE_Z) / cosf(angle);

                const auto measured_distance = zrfilter.distance_m;

                const float h[STATE_DIM] = {1/cosf(angle), 0, 0, 0, 0, 0, 0};

                return fabs(r22) > 0.1 && r22 > 0 ?
                    updateWithScalar(ekf, h,
                            measured_distance-predicted_distance,
                            zrfilter.stdev) :
                    ekf;
            }

        private:

            // State vector
            Vector x = Vector(STATE_DIM);

            // Covariance matrix
            Matrix P = Matrix(STATE_DIM, STATE_DIM);

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Vector q = Vector(4);

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            Matrix R = Matrix(STATE_VZ, 3);

            ImuSubSampler accelSubSampler;
            ImuSubSampler gyroSubSampler;

            uint32_t imuSampleCount;

            bool didResetEstimation;

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            bool isUpdated;

            uint32_t lastPredictionMs;
            uint32_t lastProcessNoiseUpdateMs;

            static auto updateWithZRange(
                    const EKF & ekf,
                    const float r22,
                    const ZRangerFilter & zrfilter) -> EKF
            {
                return ekf;
            }


            static auto addCovarianceNoise(const Matrix & P,
                    const float * noise) -> Matrix
            {
                auto Pcov = P;

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    Pcov(k,k) += noise[k] * noise[k];
                }

                return Pcov;
            }

            static auto enforceSymmetry(const Matrix & P) ->
                Matrix
                {
                    Matrix Psym = Matrix(STATE_DIM, STATE_DIM);

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

            static auto enforceSymmetry(const Vector & x)
                -> Vector
                {
                    Vector xsym = Vector(STATE_DIM);
                    xsym << x(STATE_Z), 0, 0, x(STATE_VZ), 0, 0, 0;
                    return xsym;

                }

            static auto tryToToIncorporateAttitude(
                    const Vector & q,
                    const Vector & x) -> Vector
            {
                // Incorporate the attitude error (Kalman filter state) with the attitude
                const float v0 = x(4);
                const float v1 = x(5);
                const float v2 = x(6);

                return ((isVelPositive(v0) || isVelPositive(v1) || isVelPositive(v2))
                        && (areVelsInBounds(v0, v1, v2))) ?
                    qnorm(rotate(q, v0, v1, v2))
                    : q;
            }

            static auto qnorm(const Vector & q) -> Vector
            {
                return q / (sqrt(q.cwiseProduct(q).sum()) + EPSILON);
            }

            static auto makeJacobian(
                    const Vector & x,
                    const Matrix & R,
                    const Vector & gyro,
                    const float dt) -> Matrix
            {
                const auto d0 = gyro(0)*dt/2;
                const auto d1 = gyro(1)*dt/2;
                const auto d2 = gyro(2)*dt/2;

                const auto vx = 0;
                const auto vy = 0;
                const auto vz = x(STATE_VZ);

                Matrix F(STATE_DIM, STATE_DIM);

                // position
                F(STATE_Z, STATE_Z) = 1;

                // position from body-frame velocity
                F(STATE_Z,STATE_VX) = R(2,0)*dt;
                F(STATE_Z,STATE_VY) = R(2,1)*dt;
                F(STATE_Z,STATE_VZ) = R(2,2)*dt;

                // position from attitude error
                F(STATE_Z,STATE_D0) = (vy*R(2,2) - vz*R(2,1))*dt;
                F(STATE_Z,STATE_D1) = (-vx*R(2,2) + vz*R(2,0))*dt;
                F(STATE_Z,STATE_D2) = (vx*R(2,1) - vy*R(2,0))*dt;

                // body-frame velocity from body-frame velocity
                F(STATE_VX,STATE_VX) = 1; //drag negligible
                F(STATE_VY,STATE_VX) =-gyro(2)*dt;
                F(STATE_VZ,STATE_VX) = gyro(1)*dt;

                F(STATE_VX,STATE_VY) = gyro(2)*dt;
                F(STATE_VY,STATE_VY) = 1; //drag negligible
                F(STATE_VZ,STATE_VY) =-gyro(0)*dt;

                F(STATE_VX,STATE_VZ) =-gyro(1)*dt;
                F(STATE_VY,STATE_VZ) = gyro(0)*dt;
                F(STATE_VZ,STATE_VZ) = 1; //drag negligible

                // body-frame velocity from attitude error
                F(STATE_VX,STATE_D0) =  0;
                F(STATE_VY,STATE_D0) = -G*R(2,2)*dt;
                F(STATE_VZ,STATE_D0) =  G*R(2,1)*dt;

                F(STATE_VX,STATE_D1) =  G*R(2,2)*dt;
                F(STATE_VY,STATE_D1) =  0;
                F(STATE_VZ,STATE_D1) = -G*R(2,0)*dt;

                F(STATE_VX,STATE_D2) = -G*R(2,1)*dt;
                F(STATE_VY,STATE_D2) =  G*R(2,0)*dt;
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

            static auto xinit() -> Vector
            {
                return Vector(STATE_DIM);
            }

            static auto qinit() -> Vector
            {
                auto q = Vector(4);
                q << 1, 0, 0, 0;
                return q;
            }

            static auto pinit() -> Matrix
            {
                auto P = Matrix(STATE_DIM, STATE_DIM);

                const float noise[STATE_DIM] = {
                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };

                return addCovarianceNoise(P, noise);
            }

            static auto pnoisy(const Matrix &P, const float dt)
                -> Matrix
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

                return addCovarianceNoise(P, noise);
            }

            static auto rotate(const Vector & q,
                    const float rx, const float ry, const float rz) -> Vector
            {
                const float angle = sqrt(rx*rx + ry*ry + rz*rz) + EPSILON;
                const float ca = cos(angle / 2);
                const float sa = sin(angle / 2);
                const float dq[4] = {
                    ca, sa*rx/angle, sa*ry/angle, sa*rz/angle
                };

                auto qr = Vector(4);

                qr << 
                    dq[0]*q(0) - dq[1]*q(1) - dq[2]*q(2) - dq[3]*q(3),
                    dq[1]*q(0) + dq[0]*q(1) + dq[3]*q(2) - dq[2]*q(3),
                    dq[2]*q(0) - dq[3]*q(1) + dq[0]*q(2) + dq[1]*q(3),
                    dq[3]*q(0) + dq[2]*q(1) - dq[1]*q(2) + dq[0]*q(3);

                return qr;
            }

            static auto quat2rotation(const Vector & q) -> Matrix
            {
                auto R = Matrix(3, 3);

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

            static auto areVelsInBounds(
                    const float vx, const float vy, const float vz) -> bool
            {
                return 
                    isVelInBounds(vx) &&
                    isVelInBounds(vy) &&
                    isVelInBounds(vz);
            }

            static auto isVelInBounds(const float vel) -> bool
            {
                return fabs(vel) < MAX_VELOCITY_MPS;
            }

            static auto isVelPositive(const float vel) -> bool
            {
                return fabs(vel) > MIN_VELOCITY_MPS;
            }

            static auto updateWithScalar(const EKF & ekf, const float * hvals,
                    const float error, const float stdMeasNoise) -> EKF
            {
                const auto r = stdMeasNoise * stdMeasNoise;

                auto h = Vector(STATE_DIM);

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    h(k) = hvals[k];
                }

                const auto PHT = ekf.P * h;

                const auto G = PHT / (r + PHT.dot(h));

                const auto GH = G * h.transpose();

                const auto I = Matrix::Identity(STATE_DIM, STATE_DIM);

                const auto GH_I = GH - I;

                const auto P = GH_I * ekf.P * GH_I.transpose();

                const auto x = ekf.x + G * error;

                return EKF(
                        x,
                        P,
                        ekf.q,
                        ekf.R,
                        ekf.accelSubSampler,
                        ekf.gyroSubSampler,
                        ekf.imuSampleCount,
                        ekf.didResetEstimation,
                        true, // isUpdated
                        ekf.lastPredictionMs,
                        ekf.lastProcessNoiseUpdateMs);
            }
    };
}
