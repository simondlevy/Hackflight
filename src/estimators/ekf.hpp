/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
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

#include <string.h>

#define EKF_CUSTOM
#define EKF_M 3 // range, flowx, flowy
#define EKF_N 7 // z, dx, dy, dz, e0, e1, e2
#include <tinyekf.h>
#include <tinyekf_custom.h>

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class EKF {

        public:

            void initialize(void)
            {
                const float pdiag[EKF_N] = {
                    square(STDEV_INITIAL_POSITION_Z),
                    square(STDEV_INITIAL_VELOCITY),
                    square(STDEV_INITIAL_VELOCITY),
                    square(STDEV_INITIAL_VELOCITY),
                    square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH),
                    square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH),
                    square(STDEV_INITIAL_ATTITUDE_YAW)
                };

                _isUpdated = false;

                ekf_initialize(&_ekf, pdiag);

                _quat.w = QW_INIT;
                _quat.x = QX_INIT;
                _quat.y = QY_INIT;
                _quat.z = QZ_INIT;

                _r.x = 0;
                _r.y = 0;
                _r.z = 0;
            }

            void accumulate_gyro(const axis3_t & gyro) 
            {
                imuAccum(gyro, _gyroSum);

                memcpy(&_gyroLatest, &gyro, sizeof(axis3_t));

            }

            void accumulate_accel(const axis3_t & accel) 
            {
                imuAccum(accel, _accelSum);
            }

            void predict(const float dt)
            {
                static axis3_t _gyro;
                static axis3_t _accel;

                const auto dt2 = dt * dt;

                imuTakeMean(_gyroSum, 1/Utils::RAD2DEG, _gyro);
                imuTakeMean(_accelSum, Utils::G2MSS, _accel);

                const auto xold = _ekf.x;

                // Position updates in the body frame (will be rotated to inertial
                // frame); thrust can only be produced in the body's Z direction
                const auto dx = xold[STATE_DX] * dt + _accel.x * dt2 / 2;
                const auto dy = xold[STATE_DY] * dt + _accel.y * dt2 / 2;
                const auto dz = xold[STATE_DZ] * dt + _accel.z * dt2 / 2; 

                const auto accx = _accel.x;
                const auto accy = _accel.y;

                // attitude update (rotate by gyroscope), we do this in quaternions
                // this is the gyroscope angular velocity integrated over the
                // sample period
                const auto dtwx = dt*_gyro.x;
                const auto dtwy = dt*_gyro.y;
                const auto dtwz = dt*_gyro.z;

                // compute the quaternion values in [w,x,y,z] order
                const auto angle = sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
                const auto ca = cos(angle/2);
                const auto sa = sin(angle/2);
                const auto dqw = ca;
                const auto dqx = sa*dtwx/angle;
                const auto dqy = sa*dtwy/angle;
                const auto dqz = sa*dtwz/angle;

                // rotate the quad's attitude by the delta quaternion vector
                // computed above

                const auto qw = _quat.w;
                const auto qx = _quat.x;
                const auto qy = _quat.y;
                const auto qz = _quat.z;

                const auto tmpq0 = rotateQuat(
                        dqw*qw - dqx*qx - dqy*qy - dqz*qz, QW_INIT);
                const auto tmpq1 = rotateQuat(
                        dqx*qw + dqw*qx + dqz*qy - dqy*qz, QX_INIT);
                const auto tmpq2 = rotateQuat(
                        dqy*qw - dqz*qx + dqw*qy + dqx*qz, QY_INIT);
                const auto tmpq3 = rotateQuat(
                        dqz*qw + dqy*qx - dqx*qy + dqw*qz, QZ_INIT);

                // normalize and store the result
                const auto norm = 
                    sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
                    EPS;

                // Process noise is added after the return from the prediction step

                // ====== PREDICTION STEP ======
                // The prediction depends on whether we're on the ground, or in
                // flight.  When flying, the accelerometer directly measures thrust
                // (hence is useless to estimate body angle while flying)

                const auto tmpSDX = xold[STATE_DX];
                const auto tmpSDY = xold[STATE_DY];
                const auto tmpSDZ = xold[STATE_DZ];

                const auto new_z = xold[STATE_Z] + 
                    _r.x * dx + _r.y * dy + _r.z * dz - Utils::G2MSS * dt2 / 2;

                const auto new_dx = xold[STATE_DX] +
                    dt * (accx + _gyro.z * tmpSDY - _gyro.y * tmpSDZ -
                            Utils::G2MSS * _r.x);

                const auto new_dy = xold[STATE_DY] + 
                    dt * (accy - _gyro.z * tmpSDX + _gyro.x * tmpSDZ - 
                            Utils::G2MSS * _r.y);

                const auto new_dz = xold[STATE_DZ] +
                    dt * (_accel.z + _gyro.y * tmpSDX - _gyro.x * tmpSDY - 
                            Utils::G2MSS * _r.z); 

                new_quat_t quat_predicted = {};

                quat_predicted.w = tmpq0/norm;
                quat_predicted.x = tmpq1/norm; 
                quat_predicted.y = tmpq2/norm; 
                quat_predicted.z = tmpq3/norm;

                const auto e0 = _gyro.x*dt/2;
                const auto e1 = _gyro.y*dt/2;
                const auto e2 = _gyro.z*dt/2;

                // altitude from body-frame velocity
                const auto z_dx = _r.x*dt;
                const auto z_dy = _r.y*dt;
                const auto z_dz = _r.z*dt;

                // altitude from attitude error
                const auto z_e0 = (new_dy*_r.z - new_dz*_r.y)*dt;
                const auto z_e1 = (-new_dx*_r.z + new_dz*_r.x)*dt;
                const auto z_e2 = (new_dx*_r.y - new_dy*_r.x)*dt;

                // body-frame velocity from body-frame velocity
                const auto dx_dx = 1; //drag negligible
                const auto dx_dy = _gyro.z*dt;
                const auto dx_dz = _gyro.y*dt;
                const auto dx_e0 = 0;
                const auto dx_e2 = -Utils::G2MSS*_r.y*dt;
                const auto dx_e1 = Utils::G2MSS*_r.z*dt;

                const auto dy_dx =  -_gyro.z*dt;
                const auto dy_dy = 1; //drag negligible
                const auto dy_dz = _gyro.x*dt;
                const auto dy_e0 = -Utils::G2MSS*_r.z*dt;
                const auto dy_e1 = 0;
                const auto dy_e2 = Utils::G2MSS*_r.x*dt;

                const auto dz_dx = _gyro.y*dt;
                const auto dz_dy = _gyro.x*dt;
                const auto dz_dz = 1; //drag negligible
                const auto dz_e0 = Utils::G2MSS*_r.y*dt;
                const auto dz_e1 = -Utils::G2MSS*_r.x*dt;
                const auto dz_e2 = 0;

                const auto e0_e0 =  1 - e1*e1/2 - e2*e2/2;
                const auto e0_e1 =  e2 + e0*e1/2;
                const auto e0_e2 = -e1 + e0*e2/2;

                const auto e1_e0 =  -e2 + e0*e1/2;
                const auto e1_e1 = 1 - e0*e0/2 - e2*e2/2;
                const auto e1_e2 = e0 + e1*e2/2;

                const auto e2_e0 = e1 + e0*e2/2;
                const auto e2_e1 = -e0 + e1*e2/2;
                const auto e2_e2 = 1 - e0*e0/2 - e1*e1/2;

                // Jacobian of state-transition function
                const float F[EKF_N*EKF_N] = {

                    0, z_dx,  z_dy,  z_dz,  z_e0,  z_e1,  z_e2, 
                    0, dx_dx, dx_dy, dx_dz, dx_e0, dx_e1, dx_e2, 
                    0, dy_dx, dy_dy, dy_dz, dy_e0, dy_e1, dy_e2,
                    0, dz_dx, dz_dy, dz_dz, dz_e0, dz_e1, dz_e2,
                    0, 0,     0,     0,     e0_e0, e0_e1, e0_e2,
                    0, 0,     0,     0,     e1_e0, e1_e1, e1_e2,
                    0, 0,     0,     0,     e2_e0, e2_e1, e2_e2
                };


                float fx[EKF_N] = {

                    xold[STATE_Z] ,
                    xold[STATE_DX],
                    xold[STATE_DY],
                    xold[STATE_DZ],
                    xold[STATE_E0],
                    xold[STATE_E1],
                    xold[STATE_E2]
                };

                // Avoid multiple updates within 1 msec of each other
                //static uint32_t _lastProcessNoiseUpdateMsec;
                if (true /*nowMsec - _lastProcessNoiseUpdateMsec > 0*/) {

                    //_lastProcessNoiseUpdateMsec = nowMsec;

                    fx[STATE_Z]  = new_z;
                    fx[STATE_DX] = new_dx;
                    fx[STATE_DY] = new_dy;
                    fx[STATE_DZ] = new_dz;

                    _quat.w = quat_predicted.w;
                    _quat.x = quat_predicted.x;
                    _quat.y = quat_predicted.y;
                    _quat.z = quat_predicted.z;

                    memset(&_gyroSum, 0, sizeof(_gyroSum));
                    memset(&_accelSum, 0, sizeof(_accelSum));
                }

                // We'll add process noise after final update
                const float Q[EKF_N*EKF_N] = {};

                ekf_predict(&_ekf, fx, F, Q);

                cleanupCovariance();
            }

            void update_with_range(const float distance)
            {
                const auto x = _ekf.x;

                const auto angle = max(0, 
                        fabsf(acosf(_r.z)) - 
                        (15.0f / 2.0f) / Utils::RAD2DEG);

                const auto predictedDistance = x[STATE_Z] / cosf(angle);

                const auto measuredDistance = distance / 1000.f; // mm => m
                float h[EKF_N] = {};

                h[0] = 1/cosf(angle);

                const auto r = square(RANGEFINDER_EXP_STD_A * 
                        (1 + expf(RANGEFINDER_EXP_COEFF * 
                                  (measuredDistance - RANGEFINDER_EXP_POINT_A))));

                if (fabs(_r.z) > 0.1f && _r.z > 0 && distance < RANGEFINDER_OUTLIER_LIMIT_MM) {

                    update_with_scalar(measuredDistance, predictedDistance, h, r);

                }
            }

            void update_with_flow(const float dt, const axis2_t & flow)
            {
                // Inclusion of flow measurements in the EKF done by two scalar
                // updates

                //~~~ Body rates ~~~
                const auto omegay_b = _gyroLatest.y / Utils::RAD2DEG;

                const auto x = _ekf.x;

                const auto dx_g = x[STATE_DX];

                // Saturate elevation in prediction and correction to avoid
                // singularities
                const auto z_g = x[STATE_Z] < 0.1f ? 0.1f : x[STATE_Z];

                // ~~~ X velocity prediction and update ~~~
                // predicts the number of accumulated pixels in the x-direction
                auto predictedNX = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
                    ((dx_g * _r.z / z_g) - omegay_b);
                auto measuredNX = flow.x*FLOW_RESOLUTION;

                // derive measurement equation with respect to dx (and z?)
                float hx[EKF_N] = {};
                hx[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dx_g) /
                        (-z_g * z_g));
                hx[1] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

                // Inclusion of flow measurements in the EKF done by two scalar
                // updates

                //~~~ Body rates ~~~
                const auto omegax_b = _gyroLatest.x / Utils::RAD2DEG;

                const auto dy_g = x[STATE_DY];

                // ~~~ Y velocity prediction and update ~~~
                auto predictedNY = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
                    ((dy_g * _r.z / z_g) + omegax_b);
                auto measuredNY = flow.y*FLOW_RESOLUTION;

                // derive measurement equation with respect to dy (and z?)
                float hy[EKF_N] = {};
                hy[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dy_g) / (-z_g * z_g));
                hy[2] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

                const auto r = square(FLOW_STD_FIXED * FLOW_RESOLUTION);

                update_with_scalar(measuredNX, predictedNX, hx, r);

                update_with_scalar(measuredNY, predictedNY, hy, r);
            }

            /**
             * Returns false if state is OOB, true otherwise
             */
            bool finalize()
            {
                const auto x = _ekf.x;

                // Incorporate the attitude error (Kalman filter state) with the
                // attitude
                const auto v0 = x[STATE_E0];
                const auto v1 = x[STATE_E1];
                const auto v2 = x[STATE_E2];

                const auto angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
                const auto ca = cos(angle / 2.0f);
                const auto sa = sin(angle / 2.0f);

                const auto dqw = ca;
                const auto dqx = sa * v0 / angle;
                const auto dqy = sa * v1 / angle;
                const auto dqz = sa * v2 / angle;

                const auto qw = _quat.w;
                const auto qx = _quat.x;
                const auto qy = _quat.y;
                const auto qz = _quat.z;

                // Rotate the quad's attitude by the delta quaternion vector
                // computed above
                const auto tmpq0 = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
                const auto tmpq1 = dqx * qw + dqw * qx + dqz * qy - dqy * qz;
                const auto tmpq2 = dqy * qw - dqz * qx + dqw * qy + dqx * qz;
                const auto tmpq3 = dqz * qw + dqy * qx - dqx * qy + dqw * qz;

                // normalize and store the result
                const auto norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                        tmpq3 * tmpq3) + EPS;

                const auto isErrorSufficient  = 
                    (isErrorLarge(v0) || isErrorLarge(v1) || isErrorLarge(v2)) &&
                    isErrorInBounds(v0) && isErrorInBounds(v1) && isErrorInBounds(v2);

                _quat.w = isErrorSufficient ? tmpq0 / norm : _quat.w;
                _quat.x = isErrorSufficient ? tmpq1 / norm : _quat.x;
                _quat.y = isErrorSufficient ? tmpq2 / norm : _quat.y;
                _quat.z = isErrorSufficient ? tmpq3 / norm : _quat.z;

                const float newx[EKF_N] = {
                    x[STATE_Z],
                    x[STATE_DX],
                    x[STATE_DY],
                    x[STATE_DZ],
                    0, // E0
                    0, // E1
                    0  // E2
                };

                _r.x = 2 * _quat.x * _quat.z - 2 * _quat.w * _quat.y;
                _r.y = 2 * _quat.y * _quat.z + 2 * _quat.w * _quat.x; 
                _r.z = _quat.w*_quat.w-_quat.x*_quat.x-_quat.y*_quat.y+_quat.z*_quat.z;

                // the attitude error vector (v0,v1,v2) is small,
                // so we use a first order approximation to e0 = tan(|v0|/2)*v0/|v0|
                const auto e0 = v0 / 2; 
                const auto e1 = v1 / 2; 
                const auto e2 = v2 / 2;

                const auto dx_dx = 1;
                const auto dy_dy = 1;
                const auto dz_dz = 1;

                const auto e0_e0 =  1 - e1*e1/2 - e2*e2/2;
                const auto e0_e1 =  e2 + e0*e1/2;
                const auto e0_e2 = -e1 + e0*e2/2;

                const auto e1_e0 =  -e2 + e0*e1/2;
                const auto e1_e1 = 1 - e0*e0/2 - e2*e2/2;
                const auto e1_e2 = e0 + e1*e2/2;

                const auto e2_e0 = e1 + e0*e2/2;
                const auto e2_e1 = -e0 + e1*e2/2;
                const auto e2_e2 = 1 - e0*e0/2 - e1*e1/2;

                const float A[EKF_N*EKF_N] = {
                    0, 0,     0,     0,     0,     0,     0,
                    0, dx_dx, 0,     0,     0,     0,     0,
                    0, 0,     dy_dy, 0,     0,     0,     0,
                    0, 0,     0,     dz_dz, 0,     0,     0,
                    0, 0,     0,     0,     e0_e0, e0_e1, e0_e2,
                    0, 0,     0,     0,     e1_e0, e1_e1, e1_e2,
                    0, 0,     0,     0,     e2_e0, e2_e1, e2_e2
                };

                if (_isUpdated) {

                    for (uint8_t i=0; i<EKF_N; ++i) {
                        _ekf.x[i] = newx[i];
                    }

                    if (isErrorSufficient) {

                        ekf_custom_multiply_covariance(&_ekf, A);

                        cleanupCovariance();
                    }

                    _isUpdated = false;
                }

                return
                    isPositionWithinBounds(newx[STATE_Z]) &&
                    isVelocityWithinBounds(newx[STATE_DX]) &&
                    isVelocityWithinBounds(newx[STATE_DY]) &&
                    isVelocityWithinBounds(newx[STATE_DZ]);
            }

            void get_vehicle_state(
                    axis4_t & quat, axis2_t & dxdy, float & z, float & dz)
            {
                const auto x = _ekf.x;

                dxdy.x = x[STATE_DX];

                dxdy.y = -x[STATE_DY];

                z = x[STATE_Z];

                z = min(0, z);

                dz = _r.x * x[STATE_DX] + _r.y * x[STATE_DY] + 
                    _r.z * x[STATE_DZ];

                memcpy(&quat, &_quat, sizeof(axis4_t));
           }

        private:

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_Z = 1;
            static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

            // The bounds on the covariance, these shouldn't be hit, but sometimes
            // are... why?
            static constexpr float MAX_COVARIANCE = 100;
            static constexpr float MIN_COVARIANCE = 1e-6;

            // Quaternion used for initial orientation
            static constexpr float QW_INIT = 1;
            static constexpr float QX_INIT = 0;
            static constexpr float QY_INIT = 0;
            static constexpr float QZ_INIT = 0;

            // ~~~ Camera constexprants ~~~
            // The angle of aperture is guessed from the raw data register and
            // thankfully look to be symmetric

            static constexpr float FLOW_NPIX = 35.0;   // [pixels] (same in x and y)

            // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
            // corresponding ground length
            static constexpr float FLOW_THETAPIX = 0.71674;

            //We do get the measurements in 10x the motion pixels (experimentally
            //measured)
            static constexpr float FLOW_RESOLUTION = 0.1;

            // The bounds on states, these shouldn't be hit...
            static constexpr float MAX_POSITION = 100; //meters
            static constexpr float MAX_VELOCITY = 10; //meters per second

            // Small number epsilon, to prevent dividing by zero
            static constexpr float EPS = 1e-6f;

            // the reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            static constexpr uint16_t RANGEFINDER_OUTLIER_LIMIT_MM = 5000;

            // Rangefinder measurement noise model
            static constexpr float RANGEFINDER_EXP_POINT_A = 2.5;
            static constexpr float RANGEFINDER_EXP_STD_A = 0.0025; 
            static constexpr float RANGEFINDER_EXP_POINT_B = 4.0;
            static constexpr float RANGEFINDER_EXP_STD_B = 0.2;   

            static constexpr float RANGEFINDER_EXP_COEFF = 
                logf( RANGEFINDER_EXP_STD_B / RANGEFINDER_EXP_STD_A) / 
                (RANGEFINDER_EXP_POINT_B - RANGEFINDER_EXP_POINT_A);

            static constexpr float FLOW_STD_FIXED = 2.0;

            ekf_t _ekf;

            void update_with_scalar(
                    const float z,
                    const float hx,
                    const float h[EKF_N], 
                    const float r)
            {
                ekf_custom_scalar_update(&_ekf, z, hx, h, r);

                cleanupCovariance();

                _isUpdated = true;
            }

            typedef struct {

                float w;
                float x;
                float y;
                float z;

            } new_quat_t;

            typedef struct {

                axis3_t sum;
                uint32_t count;

            } imu_t;

            axis3_t _gyroLatest;

            new_quat_t _quat;

            bool _isUpdated;

            uint32_t _nextPredictionMsec;

            axis3_t _r;

            imu_t _gyroSum;
            imu_t _accelSum;

            // Indexes to access the state
            enum {

                STATE_Z,
                STATE_DX,
                STATE_DY,
                STATE_DZ,
                STATE_E0,
                STATE_E1,
                STATE_E2
            };

            void cleanupCovariance(void)
            {
                ekf_custom_cleanup_covariance(&_ekf, MIN_COVARIANCE, MAX_COVARIANCE);
            }

            static void imuAccum(const axis3_t vals, imu_t & imu)
            {
                imu.sum.x += vals.x;
                imu.sum.y += vals.y;
                imu.sum.z += vals.z;
                imu.count++;
            }

            static void imuTakeMean(
                    const imu_t & imu, 
                    const float conversionFactor, 
                    axis3_t & mean)
            {
                const auto count = imu.count;

                const auto isCountNonzero = count > 0;

                mean.x =
                    isCountNonzero ? imu.sum.x * conversionFactor / count : mean.x;
                mean.y =
                    isCountNonzero ? imu.sum.y * conversionFactor / count : mean.y;
                mean.z =
                    isCountNonzero ? imu.sum.z * conversionFactor / count : mean.z;
            }

            static float max(const float val, const float maxval)
            {
                return val > maxval ? maxval : val;
            }

            static float min(const float val, const float maxval)
            {
                return val < maxval ? maxval : val;
            }

            static float rotateQuat(const float val, const float initVal)
            {
                return (val * (1 - ROLLPITCH_ZERO_REVERSION)) + 
                    (ROLLPITCH_ZERO_REVERSION * initVal);
            }

            static bool isPositionWithinBounds(const float pos)
            {
                return fabs(pos) < MAX_POSITION;
            }

            static bool isVelocityWithinBounds(const float vel)
            {
                return fabs(vel) < MAX_VELOCITY;
            }

            static bool isErrorLarge(const float v)
            {
                return fabs(v) > 0.1e-3f;
            }

            static bool isErrorInBounds(const float v)
            {
                return fabs(v) < 10;
            }

            static float square(const float x)
            {
                return x * x;
            }
    };

}
