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
#include <firmware/ekf/vec3_subsampler.hpp>
#include <firmware/opticalflow.hpp>
#include <firmware/timer.hpp>
#include <firmware/zranger.hpp>
#include <num.hpp>

#include <ArduinoEigenDense.h>

// We want to use F for the Jacobian, but Arduino pre-defines it
#ifdef F
#undef F
#endif

namespace hf {

    class EKF { 

        private:

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_XY = 100;
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

            // Small number epsilon, to prevent dividing by zero
            static constexpr float EPSILON = 1e-6f;

            // the reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            static const size_t QUEUE_MAX_LENGTH = 20;

        public:

            EKF()
            {
                reset(0);
            }

            EKF& operator=(const EKF& other) = default;

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
                    predict(msec_curr, isFlying); 
                    _msec_prev = msec_curr;
                }

                addProcessNoise(msec_curr);

                // Update with queued measurements and flush the queue
                for (uint32_t k=0; k<_queueLength; ++k) {
                    update(_measurementsQueue[k], msec_curr);
                }
                _queueLength = 0;

                const auto z = _x(0);

                if (_isUpdated) {
                    finalize(msec_curr);
                }

                const auto dx = _r00*_x(1) + _r01*_x(2) + _r02*_x(3);

                // make right positive
                const auto dy = -(_r10*_x(1) + _r11*_x(2) + _r12*_x(3)); 

                const auto dz = _r20*_x(1) + _r21*_x(2) + _r22*_x(3);

                const auto phi = Num::RAD2DEG * atan2f(2*(_q(2)*_q(3)+_q(0)* _q(1)) ,
                        _q(0)*_q(0) - _q(1)*_q(1) - _q(2)*_q(2) + _q(3)*_q(3));

                const auto dphi = _gyroLatest.x;

                const auto theta = Num::RAD2DEG * asinf(-2*(_q(1)*_q(3) - _q(0)*_q(2)));

                const auto dtheta = _gyroLatest.y;

                const auto psi = Num::RAD2DEG * atan2f(2*(_q(1)*_q(2)+_q(0)* _q(3)),
                        _q(0)*_q(0) + _q(1)*_q(1) - _q(2)*_q(2) - _q(3)*_q(3)); 

                const auto dpsi = _gyroLatest.z;

                return VehicleState(dx, dy, z, dz, phi, dphi, theta, dtheta,
                        -psi, -dpsi); // make nose-right positive
            }

            void enqueueImu(const ImuFiltered & imu)
            {
                measurement_t m = {};
                m.type = MeasurementTypeGyroscope;
                m.data.gyroscope.gyro = imu.gyroDps;
                enqueue(&m);

                m = {};
                m.type = MeasurementTypeAcceleration;
                m.data.acceleration.acc = imu.accelGs;
                enqueue(&m);
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
                MeasurementTypeAcceleration,
                MeasurementTypeGyroscope,
                MeasurementTypeTOF,
                MeasurementTypeFlow,
            } MeasurementType;

            typedef struct {
                Vec3 gyro; // deg/s
            } gyroscopeMeasurement_t;

            typedef struct {
                Vec3 acc; // Gs
            } accelerationMeasurement_t;

            typedef struct {
                MeasurementType type;
                union
                {
                    gyroscopeMeasurement_t gyroscope;
                    accelerationMeasurement_t acceleration;
                    ZRanger::measurement_t tof;
                    OpticalFlow::measurement_t flow;
                } data;
            } measurement_t;

            static const auto QUEUE_ITEM_SIZE = sizeof(EKF::measurement_t);

            // Instance vars --------------------------------------------------

            EKF::measurement_t _measurementsQueue[QUEUE_MAX_LENGTH];
            uint32_t _queueLength;

            ImuFiltered _imuQueue[QUEUE_MAX_LENGTH];

            // State vector
            __attribute__((aligned(4))) Eigen::VectorXd _x =
                Eigen::VectorXd(STATE_DIM);

            // Covariance matrix
            Eigen::MatrixXd P = Eigen::MatrixXd(STATE_DIM, STATE_DIM);

            bool _didResetEstimation;

            uint32_t _msec_prev;

            Vec3 _accLatest;
            Vec3 _gyroLatest;

            Vec3SubSampler _accelSubSampler;
            Vec3SubSampler _gyroSubSampler;

            float _predictedNX;
            float _predictedNY;

            float _measuredNX;
            float _measuredNY;

            // The vehicle's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            float _r00, _r01, _r02, _r10, _r11, _r12, _r20, _r21, _r22; 

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Eigen::VectorXd _q = Eigen::VectorXd(4);

            // Quaternion used for initial orientation
            Eigen::VectorXd _qinit = Eigen::VectorXd(4);
            
            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            bool _isUpdated;

            uint32_t _lastPredictionMs;
            uint32_t _lastProcessNoiseUpdateMs;

            // Instance methods ----------------------------------------------

            void reset(const uint32_t msec_curr)
            {
                _accelSubSampler = Vec3SubSampler(GRAVITY);
                _gyroSubSampler = Vec3SubSampler(Num::DEG2RAD);

                ekf_init();

                // Reset the attitude quaternion
                _q(0) = _qinit(0) = 1;
                _q(1) = _qinit(1) = 0;
                _q(2) = _qinit(2) = 0;
                _q(3) = _qinit(3) = 0;

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

                _isUpdated = false;
                _lastPredictionMs = msec_curr;
                _lastProcessNoiseUpdateMs = msec_curr;
            }

            void predict(const uint32_t msec_curr, bool isFlying) 
            {
                _accelSubSampler = Vec3SubSampler::finalize(_accelSubSampler);
                _gyroSubSampler = Vec3SubSampler::finalize(_gyroSubSampler);

                const float dt = (msec_curr - _lastPredictionMs) / 1000.0f;

                const Vec3 * accel = &_accelSubSampler.subSample;
                const Vec3 * gyro = &_gyroSubSampler.subSample;

                const float d0 = gyro->x*dt/2;
                const float d1 = gyro->y*dt/2;
                const float d2 = gyro->z*dt/2;

                const float vx = _x(1);
                const float vy = _x(2);
                const float vz = _x(3);

                static Eigen::MatrixXd F(7, 7);

                // position
                F(0, 0) = 1;

                // position from body-frame velocity
                F(0,1) = _r20*dt;

                F(0,2) = _r21*dt;

                F(0,3) = _r22*dt;

                // position from attitude error
                F(0,4) = (vy*_r22 - vz*_r21)*dt;

                F(0,5) = (-vx*_r22 + vz*_r20)*dt;

                F(0,6) = (vx*_r21 - vy*_r20)*dt;

                // body-frame velocity from body-frame velocity
                F(1,1) = 1; //drag negligible
                F(2,1) =-gyro->z*dt;
                F(3,1) = gyro->y*dt;

                F(1,2) = gyro->z*dt;
                F(2,2) = 1; //drag negligible
                F(3,2) =-gyro->x*dt;

                F(1,3) =-gyro->y*dt;
                F(2,3) = gyro->x*dt;
                F(3,3) = 1; //drag negligible

                // body-frame velocity from attitude error
                F(1,4) =  0;
                F(2,4) = -GRAVITY*_r22*dt;
                F(3,4) =  GRAVITY*_r21*dt;

                F(1,5) =  GRAVITY*_r22*dt;
                F(2,5) =  0;
                F(3,5) = -GRAVITY*_r20*dt;

                F(1,6) = -GRAVITY*_r21*dt;
                F(2,6) =  GRAVITY*_r20*dt;
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

                // P_k = F_{k-1} P_{k-1} F^T_{k-1}
                P = (F * P) * F.transpose();

                const float dt2 = dt * dt;

                // keep previous time step's state for the update
                const float tmpSPX = _x(1);
                const float tmpSPY = _x(2);
                const float tmpSPZ = _x(3);

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _x(1) * dt + (isFlying ? 0 : accel->x * dt2 / 2.0f);
                const float dy = _x(2) * dt + (isFlying ? 0 : accel->y * dt2 / 2.0f);

                // thrust can only be produced in the body's Z direction
                const float dz = _x(3) * dt + accel->z * dt2 / 2.0f; 

                // position update
                _x(0) += _r20 * dx + _r21 * dy + _r22 * dz - 
                    GRAVITY * dt2 / 2.0f;

                const float accelx = isFlying ? 0 : accel->x;
                const float accely = isFlying ? 0 : accel->y;

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame

                _x(1) += dt * (accelx + gyro->z * tmpSPY - gyro->y * tmpSPZ
                        - GRAVITY * _r20);

                _x(2) += dt * (accely - gyro->z * tmpSPX + gyro->x * tmpSPZ
                        - GRAVITY * _r21);

                _x(3) += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY
                        - GRAVITY * _r22);

                // attitude update (rotate by gyroscope), we do this in quaternions
                // this is the gyroscope angular velocity integrated over the sample period
                const float dtwx = dt*gyro->x;
                const float dtwy = dt*gyro->y;
                const float dtwz = dt*gyro->z;

                // compute the quaternion values in [w,x,y,z] order
                const float angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPSILON;
                const float ca = cos(angle/2.0f);
                const float sa = sin(angle/2.0f);
                const float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

                // rotate the vehicle's attitude by the delta quaternion vector computed above

                float tmpq0 = dq[0]*_q(0) - dq[1]*_q(1) - dq[2]*_q(2) - dq[3]*_q(3);
                float tmpq1 = dq[1]*_q(0) + dq[0]*_q(1) + dq[3]*_q(2) - dq[2]*_q(3);
                float tmpq2 = dq[2]*_q(0) - dq[3]*_q(1) + dq[0]*_q(2) + dq[1]*_q(3);
                float tmpq3 = dq[3]*_q(0) + dq[2]*_q(1) - dq[1]*_q(2) + dq[0]*_q(3);

                if (!isFlying) {

                    const float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                    tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * _qinit(0);
                    tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * _qinit(1);
                    tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * _qinit(2);
                    tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * _qinit(3);
                }

                // normalize and store the result
                const float norm = sqrt(
                        tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPSILON;

                _q(0) = tmpq0/norm; 
                _q(1) = tmpq1/norm; 
                _q(2) = tmpq2/norm; 
                _q(3) = tmpq3/norm;

                _isUpdated = true;
                _lastPredictionMs = msec_curr;
            }

            void enqueue(const EKF::measurement_t * measurement) 
            {
                memcpy(&_measurementsQueue[_queueLength], measurement,
                        sizeof(EKF::measurement_t));
                _queueLength = (_queueLength + 1) % QUEUE_MAX_LENGTH;
            }

            void finalize(const uint32_t msec_curr)
            {
                // Incorporate the attitude error (Kalman filter state) with the attitude
                const float v0 = _x(4);
                const float v1 = _x(5);
                const float v2 = _x(6);

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
                    const float tmpq0 = dq[0] * _q(0) - dq[1] * _q(1) - 
                        dq[2] * _q(2) - dq[3] * _q(3);
                    const float tmpq1 = dq[1] * _q(0) + dq[0] * _q(1) + 
                        dq[3] * _q(2) - dq[2] * _q(3);
                    const float tmpq2 = dq[2] * _q(0) - dq[3] * _q(1) + 
                        dq[0] * _q(2) + dq[1] * _q(3);
                    const float tmpq3 = dq[3] * _q(0) + dq[2] * _q(1) - 
                        dq[1] * _q(2) + dq[0] * _q(3);

                    // normalize and store the result
                    float norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                            tmpq3 * tmpq3) + EPSILON;
                    _q(0) = tmpq0 / norm;
                    _q(1) = tmpq1 / norm;
                    _q(2) = tmpq2 / norm;
                    _q(3) = tmpq3 / norm;
                }

                // Convert the new attitude to a rotation matrix, such that we can
                // rotate body-frame velocity and acc

                _r00 = _q(0) * _q(0) + _q(1) * _q(1) - _q(2) * _q(2) - _q(3) * _q(3);
                _r01 = 2 * _q(1) * _q(2) - 2 * _q(0) * _q(3);
                _r02 = 2 * _q(1) * _q(3) + 2 * _q(0) * _q(2);
                _r10 = 2 * _q(1) * _q(2) + 2 * _q(0) * _q(3);
                _r11 = _q(0) * _q(0) - _q(1) * _q(1) + _q(2) * _q(2) - _q(3) * _q(3);
                _r12 = 2 * _q(2) * _q(3) - 2 * _q(0) * _q(1);
                _r20 = 2 * _q(1) * _q(3) - 2 * _q(0) * _q(2);
                _r21 = 2 * _q(2) * _q(3) + 2 * _q(0) * _q(1);
                _r22 = _q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3);

                // reset the attitude error
                _x(4) = 0;
                _x(5) = 0;
                _x(6) = 0;

                ekf_enforceSymmetry();

                _isUpdated = false;
            }

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

            void update(measurement_t & m, const uint32_t msec_curr)
            {
                switch (m.type) {

                    case MeasurementTypeGyroscope:
                        updateWithGyro(m);
                        break;

                    case MeasurementTypeAcceleration:
                        updateWithAccel(m);
                        break;

                    default:
                        break;
                }
            }

            void updateWithAccel(measurement_t & m)
            {
                _accelSubSampler = Vec3SubSampler::accumulate(_accelSubSampler,
                        m.data.acceleration.acc);
                _accLatest = m.data.acceleration.acc;
            }

            void updateWithGyro(measurement_t & m)
            {
                _gyroSubSampler = Vec3SubSampler::accumulate(_gyroSubSampler,
                        m.data.gyroscope.gyro);
                _gyroLatest = m.data.gyroscope.gyro;
            }

            void ekf_init()
            {
                _x = Eigen::VectorXd(7);

                P = Eigen::MatrixXd(7, 7);
            }

            void ekf_addCovarianceNoise(const float * noise)
            {
                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    P(k,k) += noise[k] * noise[k];
                }
            }

            void ekf_enforceSymmetry()
            {
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        const auto pval = (P(i,j) + P(j,i)) / 2;

                        P(i,j) = P(j,i) = 
                            isnan(pval) || pval > MAX_COVARIANCE ? MAX_COVARIANCE :
                            i==j && pval < MIN_COVARIANCE ? MIN_COVARIANCE :
                            pval;
                     }
                }
            }
    };
}
