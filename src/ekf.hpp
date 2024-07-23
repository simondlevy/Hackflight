#pragma once

#include <math.h>
#include <string.h>

#include <datatypes.h>
#include <utils.hpp>

#define EKF_CUSTOM
#define EKF_M 3 // range, flowx, flowy
#define EKF_N 7 // z, dx, dy, dz, e0, e1, e2
#include <tinyekf.h>
#include <tinyekf_custom.h>

class Ekf {

    public:

        void initialize(void)
        {
            const float pdiag[7] = {
                Utils::square(STDEV_INITIAL_POSITION_Z),
                Utils::square(STDEV_INITIAL_VELOCITY),
                Utils::square(STDEV_INITIAL_VELOCITY),
                Utils::square(STDEV_INITIAL_VELOCITY),
                Utils::square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH),
                Utils::square(STDEV_INITIAL_ATTITUDE_ROLL_PITCH),
                Utils::square(STDEV_INITIAL_ATTITUDE_YAW)
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

        /**
         * Input: gyro values in degrees / sec
         */
        void accumulate_gyro(const axis3_t & gyro)
        {
            imuAccum(gyro, _gyroSum);

            _gyroLatest.x = gyro.x;
            _gyroLatest.y = gyro.y;
            _gyroLatest.z = gyro.z;
        }

        /**
         * Input: accel values in gs
         */
        void accumulate_accel(const axis3_t & accel)
        {
            imuAccum(accel, _accelSum);
        }

        void predict(const uint32_t nowMsec)
        {
            // Compute DT
            static uint32_t _lastPredictionMsec;
            const float dt = (nowMsec - _lastPredictionMsec) / 1000.0f;
            _lastPredictionMsec = nowMsec;

            const auto dt2 = dt * dt;

            axis3_t gyro = {};
            imuTakeMean(_gyroSum, Utils::DEG2RAD, gyro);

            axis3_t accel = {};
            imuTakeMean(_accelSum, Utils::GS2MSS, accel);

            // Position updates in the body frame (will be rotated to inertial frame);
            // thrust can only be produced in the body's Z direction
            const auto dx = _ekf.x[STATE_DX] * dt + accel.x * dt2 / 2;
            const auto dy = _ekf.x[STATE_DY] * dt + accel.y * dt2 / 2;
            const auto dz = _ekf.x[STATE_DZ] * dt + accel.z * dt2 / 2; 

            // Process noise is added after the return from the prediction step

            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust
            // (hence is useless to estimate body angle while flying)

            const auto tmpSDX = _ekf.x[STATE_DX];
            const auto tmpSDY = _ekf.x[STATE_DY];
            const auto tmpSDZ = _ekf.x[STATE_DZ];

            const auto new_z = _ekf.x[STATE_Z] + 
                _r.x * dx + _r.y * dy + _r.z * dz - Utils::GS2MSS * dt2 / 2;

            const auto new_dx = _ekf.x[STATE_DX] +
                dt * (accel.x + gyro.z * tmpSDY - gyro.y * tmpSDZ -
                        Utils::GS2MSS * _r.x);

            const auto new_dy = _ekf.x[STATE_DY] + 
                dt * (accel.y - gyro.z * tmpSDX + gyro.x * tmpSDZ - 
                        Utils::GS2MSS * _r.y);

            const auto new_dz = _ekf.x[STATE_DZ] +
                dt * (accel.z + gyro.y * tmpSDX - gyro.x * tmpSDY - 
                        Utils::GS2MSS * _r.z); 

            // Jacobian of state-transition function
            float F[EKF_N*EKF_N] = {};

            // Integrate gyro to approximate angle
            fillMatrix(F, gyro.x*dt, gyro.y*dt, gyro.z*dt);

            setMatrixElement(F, STATE_Z, STATE_DX, _r.x*dt);
            setMatrixElement(F, STATE_Z, STATE_DY, _r.y*dt);
            setMatrixElement(F, STATE_Z, STATE_DZ, _r.z*dt);

            setMatrixElement(F, STATE_Z, STATE_E0, (new_dy*_r.z - new_dz*_r.y)*dt);
            setMatrixElement(F, STATE_Z, STATE_E1, (-new_dx*_r.z + new_dz*_r.x)*dt);
            setMatrixElement(F, STATE_Z, STATE_E2, (new_dx*_r.y - new_dy*_r.x)*dt);

            setMatrixElement(F, STATE_DX, STATE_DY, gyro.z*dt);
            setMatrixElement(F, STATE_DX, STATE_DZ, gyro.y*dt);
            setMatrixElement(F, STATE_DX, STATE_E2, -Utils::GS2MSS*_r.y*dt);
            setMatrixElement(F, STATE_DX, STATE_E1, Utils::GS2MSS*_r.z*dt);

            setMatrixElement(F, STATE_DY, STATE_DX,  -gyro.z*dt);
            setMatrixElement(F, STATE_DY, STATE_DZ, gyro.x*dt);
            setMatrixElement(F, STATE_DY, STATE_E0, -Utils::GS2MSS*_r.z*dt);
            setMatrixElement(F, STATE_DY, STATE_E2, Utils::GS2MSS*_r.x*dt);

            setMatrixElement(F, STATE_DZ, STATE_DX, gyro.y*dt);
            setMatrixElement(F, STATE_DZ, STATE_DY, gyro.x*dt);
            setMatrixElement(F, STATE_DZ, STATE_E0, Utils::GS2MSS*_r.y*dt);
            setMatrixElement(F, STATE_DZ, STATE_E1, -Utils::GS2MSS*_r.x*dt);

            float fx[EKF_N] = {

                _ekf.x[STATE_Z] ,
                _ekf.x[STATE_DX],
                _ekf.x[STATE_DY],
                _ekf.x[STATE_DZ],
                _ekf.x[STATE_E0],
                _ekf.x[STATE_E1],
                _ekf.x[STATE_E2]
            };

            // Avoid multiple updates within 1 msec of each other
            static uint32_t _lastProcessNoiseUpdateMsec;
            if (nowMsec - _lastProcessNoiseUpdateMsec > 0) {

                _lastProcessNoiseUpdateMsec = nowMsec;

                fx[STATE_Z]  = new_z;
                fx[STATE_DX] = new_dx;
                fx[STATE_DY] = new_dy;
                fx[STATE_DZ] = new_dz;

                // Integrate gyro to predict quaternion
                computeQuaternion(dt * gyro.x, dt * gyro.y, dt * gyro.z);

                memset(&_gyroSum, 0, sizeof(_gyroSum));
                memset(&_accelSum, 0, sizeof(_accelSum));
            }

            // We'll add process noise after final update
            const float Q[EKF_N*EKF_N] = {};

            ekf_predict(&_ekf, fx, F, Q);

            cleanupCovariance();

        } // predict

        void update_with_range(const float distance)
        {
            const auto x = _ekf.x;

            const auto angle = Utils::fmax(0, 
                    fabsf(acosf(_r.z)) - 
                    Utils::DEG2RAD * (15.0f / 2.0f));

            const auto predictedDistance = x[STATE_Z] / cosf(angle);

            const auto measuredDistance = distance / 1000.f; // mm => m
            float h[7] = {};

            h[0] = 1/cosf(angle);

            const auto r = Utils::square(RANGEFINDER_EXP_STD_A * 
                    (1 + expf(RANGEFINDER_EXP_COEFF * 
                              (measuredDistance - RANGEFINDER_EXP_POINT_A))));

            if (fabs(_r.z) > 0.1f && _r.z > 0 && 
                    distance < RANGEFINDER_OUTLIER_LIMIT_MM) {

                update_with_scalar(measuredDistance, predictedDistance, h, r);
            }
        }

        void update_with_flow(const float dt, const float dx, const float dy)
        {
            // Inclusion of flow measurements in the EKF done by two scalar updates

            //~~~ Body rates ~~~
            const auto omegay_b = _gyroLatest.y * Utils::DEG2RAD;

            const auto x = _ekf.x;

            const auto dx_g = x[STATE_DX];

            // Saturate elevation in prediction and correction to avoid singularities
            const auto z_g = x[STATE_Z] < 0.1f ? 0.1f : x[STATE_Z];

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            auto predictedNX = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
                ((dx_g * _r.z / z_g) - omegay_b);
            auto measuredNX = dx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            float hx[EKF_N] = {};
            hx[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dx_g) / (-z_g * z_g));
            hx[1] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

            // Inclusion of flow measurements in the EKF done by two scalar updates

            //~~~ Body rates ~~~
            const auto omegax_b = _gyroLatest.x * Utils::DEG2RAD;

            const auto dy_g = x[STATE_DY];

            // ~~~ Y velocity prediction and update ~~~
            auto predictedNY = (dt * FLOW_NPIX / FLOW_THETAPIX ) * 
                ((dy_g * _r.z / z_g) + omegax_b);
            auto measuredNY = dy*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            float hy[EKF_N] = {};
            hy[0] = (FLOW_NPIX * dt / FLOW_THETAPIX) * ((_r.z * dy_g) / (-z_g * z_g));
            hy[2] = (FLOW_NPIX * dt / FLOW_THETAPIX) * (_r.z / z_g);

            const auto r = Utils::square(FLOW_STD_FIXED * FLOW_RESOLUTION);

            update_with_scalar(measuredNX, predictedNX, hx, r);

            update_with_scalar(measuredNY, predictedNY, hy, r);
        }

        bool finalize(void)
        {
            // Incorporate the attitude error (Kalman filter state) with the attitude
            const auto e0 = _ekf.x[STATE_E0];
            const auto e1 = _ekf.x[STATE_E1];
            const auto e2 = _ekf.x[STATE_E2];

            const auto isErrorSufficient  = 
                (isErrorLarge(e0) || isErrorLarge(e1) || isErrorLarge(e2)) &&
                isErrorInBounds(e0) && isErrorInBounds(e1) && isErrorInBounds(e2);

            // Update quaternion iff attitude error is sufficient
            if (isErrorSufficient) {
                computeQuaternion(e0, e1, e2);
            }

            const float newx[EKF_N] = {
                _ekf.x[STATE_Z],
                _ekf.x[STATE_DX],
                _ekf.x[STATE_DY],
                _ekf.x[STATE_DZ],
                0, // E0
                0, // E1
                0  // E2
            };

            _r.x = 2 * _quat.x * _quat.z - 2 * _quat.w * _quat.y;
            _r.y = 2 * _quat.y * _quat.z + 2 * _quat.w * _quat.x; 
            _r.z = _quat.w*_quat.w-_quat.x*_quat.x-_quat.y*_quat.y+_quat.z*_quat.z;

            if (_isUpdated) {

                for (uint8_t i=0; i<EKF_N; ++i) {
                    _ekf.x[i] = newx[i];
                }

                if (isErrorSufficient) {

                    float A[EKF_N*EKF_N] = {};

                    fillMatrix(A, e0, e1, e2);

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

        } // finalize

        void get_vehicle_state(quat_t & quat, axis3_t & pos, axis3_t & dpos)
        {
            memcpy(&quat, &_quat, sizeof (quat_t));

            const auto x = _ekf.x;

            dpos.x = x[STATE_DX];

            // Negate for rightward positive
            dpos.y = -x[STATE_DY];

            pos.z = x[STATE_Z];

            pos.z = Utils::fmin(0, pos.z);

            dpos.z = _r.x * x[STATE_DX] + _r.y * x[STATE_DY] + 
                _r.z * x[STATE_DZ];
        }

    private:

        // Initial variances, uncertain of position, but know we're
        // stationary and roughly flat
        static constexpr float STDEV_INITIAL_POSITION_Z = 1;
        static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
        static constexpr float STDEV_INITIAL_ATTITUDE_ROLL_PITCH = 0.01;
        static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

        // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
        static constexpr float MAX_COVARIANCE = 100;
        static constexpr float MIN_COVARIANCE = 1e-6;

        // Quaternion used for initial orientation
        static constexpr float QW_INIT = 1;
        static constexpr float QX_INIT = 0;
        static constexpr float QY_INIT = 0;
        static constexpr float QZ_INIT = 0;

        // The angle of aperture is guessed from the raw data register and
        // thankfully look to be symmetric

        static constexpr float FLOW_NPIX = 35.0;   // [pixels] (same in x and y)

        // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
        // corresponding ground length
        static constexpr float FLOW_THETAPIX = 0.71674;

        // We do get the measurements in 10x the motion pixels (experimentally measured)
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

        // Indices to access the state
        enum {

            STATE_Z,
            STATE_DX,
            STATE_DY,
            STATE_DZ,
            STATE_E0,
            STATE_E1,
            STATE_E2
        };

        typedef struct {

            axis3_t sum;
            uint32_t count;

        } imu_t;

        ekf_t _ekf;

        axis3_t _r;

        bool _isUpdated;

        imu_t _gyroSum;
        imu_t _accelSum;

        quat_t _quat;

        axis3_t _gyroLatest;

        void cleanupCovariance(void)
        {
            ekf_custom_cleanup_covariance(&_ekf, MIN_COVARIANCE, MAX_COVARIANCE);
        }

        static void imuAccum(const axis3_t values, imu_t & imu)
        {
            imu.sum.x += values.x;
            imu.sum.y += values.y;
            imu.sum.z += values.z;
            imu.count++;
        }

        static void imuTakeMean(
                const imu_t & imu, 
                const float conversionFactor, 
                axis3_t & mean)
        {
            const auto count = imu.count;

            const auto isCountNonzero = count > 0;

            mean.x = isCountNonzero ? imu.sum.x * conversionFactor / count : mean.x;
            mean.y = isCountNonzero ? imu.sum.y * conversionFactor / count : mean.y;
            mean.z = isCountNonzero ? imu.sum.z * conversionFactor / count : mean.z;
        }

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

        void computeQuaternion(const float x,  const float y,  const float z)
        {
            const auto angle = sqrt(x * x + y * y + z * z) + EPS;
            const auto ca = cos(angle / 2);
            const auto sa = sin(angle / 2);
            const auto dqw = ca;
            const auto dqx = sa * x / angle;
            const auto dqy = sa * y / angle;
            const auto dqz = sa * z / angle;

            const auto qw = _quat.w;
            const auto qx = _quat.x;
            const auto qy = _quat.y;
            const auto qz = _quat.z;

            const auto tmpq0 = rotateQuat(
                    dqw * qw - dqx * qx - dqy * qy - dqz * qz, QW_INIT);
            const auto tmpq1 = rotateQuat(
                    dqx * qw + dqw * qx + dqz * qy - dqy * qz, QX_INIT);
            const auto tmpq2 = rotateQuat(
                    dqy * qw - dqz * qx + dqw * qy + dqx * qz, QY_INIT);
            const auto tmpq3 = rotateQuat(
                    dqz * qw + dqy * qx - dqx * qy + dqw * qz, QZ_INIT);

            // normalize and store the result
            const auto norm = 
                sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + 
                EPS;

            _quat.w = tmpq0 / norm;
            _quat.x = tmpq1 / norm;
            _quat.y = tmpq2 / norm;
            _quat.z = tmpq3 / norm;
        }

        static void fillMatrix(
                float A[EKF_N*EKF_N], 
                const float x, 
                const float y, 
                const float z)
        {
            const auto e0 = x / 2;
            const auto e1 = y / 2;
            const auto e2 = z / 2;

            setMatrixElement(A, STATE_DX, STATE_DX, 1);
            setMatrixElement(A, STATE_DY, STATE_DY, 1);
            setMatrixElement(A, STATE_DZ, STATE_DZ, 1);

            // The attitude error vector (v0,v1,v2) is small, so we use a
            // first-order approximation to e0 = tan(|v0|/2)*v0/|v0|
            setMatrixElement(A, STATE_E0, STATE_E0, 1 - e1*e1/2 - e2*e2/2);
            setMatrixElement(A, STATE_E0, STATE_E1, e2 + e0*e1/2);
            setMatrixElement(A, STATE_E0, STATE_E2, -e1 + e0*e2/2);
            setMatrixElement(A, STATE_E1, STATE_E0, -e2 + e0*e1/2);
            setMatrixElement(A, STATE_E1, STATE_E1, 1 - e0*e0/2 - e2*e2/2);
            setMatrixElement(A, STATE_E1, STATE_E2, e0 + e1*e2/2);
            setMatrixElement(A, STATE_E2, STATE_E0, e1 + e0*e2/2);
            setMatrixElement(A, STATE_E2, STATE_E1, -e0 + e1*e2/2);
            setMatrixElement(A, STATE_E2, STATE_E2, 1 - e0*e0/2 - e1*e1/2);
        }

        static void setMatrixElement(float A[EKF_N*EKF_N], 
                const uint8_t row, const uint8_t col, const float val)
        {
            A[row * EKF_N + col] = val;
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
};
