/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * @INPROCEEDINGS{MuellerHamerUWB2015,
 * author = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
 * title  = {Fusing ultra-wideband range measurements with accelerometers and rate 
 * gyroscopes for quadrocopter state estimation},
 * booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
 * year   = {2015},
 * month  = {May},
 * pages  = {1730-1736},
 * doi    = {10.1109/ICRA.2015.7139421},
 * ISSN   = {1050-4729}}
 *
 * @ARTICLE{MuellerCovariance2016,
 * author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
 * title={Covariance Correction Step for Kalman Filtering with an Attitude},
 * journal={Journal of Guidance, Control, and Dynamics},
 * pages={1--7},
 * year={2016},
 * publisher={American Institute of Aeronautics and Astronautics}}
 */

#pragma once

#include <math3d.h>
#include <outlierFilterTdoa.hpp>
#include <datatypes.h>
#include <matrix_typedef.h>

#define EKF_N 9
#include <ekf.hpp>

class KalmanFilter { 

    private:

        // Indexes to access the vehicle's state, stored as a column vector
        enum
        {
            STATE_X,
            STATE_Y,
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

        typedef enum {
            MeasurementTypeAcceleration,
            MeasurementTypeGyroscope,
            MeasurementTypeTOF,
            MeasurementTypeFlow,
        } MeasurementType;

        typedef struct
        {
            MeasurementType type;
            union
            {
                gyroscopeMeasurement_t gyroscope;
                accelerationMeasurement_t acceleration;
                tofMeasurement_t tof;
                flowMeasurement_t flow;
            } data;
        } measurement_t;

        void init(const uint32_t nowMs)
        {
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY_MAGNITUDE);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            _outlierFilterTdoa.reset();

            _state_vector[STATE_X] = 0;
            _state_vector[STATE_Y] = 0;
            _state_vector[STATE_Z] = 0;

            // Reset the attitude quaternion
            _initialQuaternion[0] = 1;
            _initialQuaternion[1] = 0;
            _initialQuaternion[2] = 0;
            _initialQuaternion[3] = 0;

            for (int i = 0; i < 4; i++) { 
                _quat[i] = _initialQuaternion[i]; 
            }

            // Set the initial rotation matrix to the identity. This only affects
            // the first prediction step, since in the finalization, after shifting
            // attitude errors into the attitude state, the rotation matrix is updated.
            for(int i=0; i<3; i++) { 
                for(int j=0; j<3; j++) { 
                    _rotmat[i][j] = i==j ? 1 : 0; 
                }
            }

            // Set the covariance matrix to zero
            for (int i=0; i< STATE_DIM; i++) {
                for (int j=0; j < STATE_DIM; j++) {
                    _Pmatrix[i][j] = 0; 
                }
            }

            // Add in the initial process noise 
            const float pinit[STATE_DIM] = {

                STDEV_INITIAL_POSITION_XY,
                STDEV_INITIAL_POSITION_XY,
                STDEV_INITIAL_POSITION_Z,
                STDEV_INITIAL_VELOCITY,
                STDEV_INITIAL_VELOCITY,
                STDEV_INITIAL_VELOCITY,
                STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                STDEV_INITIAL_ATTITUDE_YAW
            };
            addCovarianceNoise(pinit);

            _ekf.init(pinit, nowMs, MIN_COVARIANCE, MAX_COVARIANCE);

            _Pmatrix_m.numRows = STATE_DIM;
            _Pmatrix_m.numCols = STATE_DIM;
            _Pmatrix_m.pData = (float*)_Pmatrix;

            _isUpdated = false;
            _lastPredictionMs = nowMs;
            _lastProcessNoiseUpdateMs = nowMs;
        }

        void predict(const uint32_t nowMs, bool isFlying) 
        {
            axis3fSubSamplerFinalize(&_accSubSampler);
            axis3fSubSamplerFinalize(&_gyroSubSampler);

            const float dt = (nowMs - _lastPredictionMs) / 1000.0f;

            Axis3f * accel = &_accSubSampler.subSample;
            Axis3f * gyro = &_gyroSubSampler.subSample;

            // The linearized update matrix
            static float A[STATE_DIM][STATE_DIM];

            /* Here we discretize (euler forward) and linearise the quadrocopter
             * dynamics in order to push the covariance forward.
             *
             * QUADROCOPTER DYNAMICS (see paper):
             *
             * \dot{x} = R(I + [[d]])p
             * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
             * \dot{d} = \omega
             *
             * where [[.]] is the cross-product matrix of .
             *       \omega are the gyro measurements
             *       e3 is the column vector [0 0 1]'
             *       I is the identity
             *       R is the current attitude as a rotation matrix
             *       f/m is the mass-normalized motor force (acceleration in the body's z 
             direction)
             *       g is gravity
             *       x, p, d are the vehicle's states
             * note that d (attitude error) is zero at the beginning of each iteration,
             * since error information is incorporated into R after each Kalman update.
             */

            // ====== DYNAMICS LINEARIZATION ======
            // Initialize as the identity
            A[STATE_X][STATE_X] = 1;
            A[STATE_Y][STATE_Y] = 1;
            A[STATE_Z][STATE_Z] = 1;

            A[STATE_VX][STATE_VX] = 1;
            A[STATE_VY][STATE_VY] = 1;
            A[STATE_VZ][STATE_VZ] = 1;

            A[STATE_D0][STATE_D0] = 1;
            A[STATE_D1][STATE_D1] = 1;
            A[STATE_D2][STATE_D2] = 1;

            // position from body-frame velocity
            A[STATE_X][STATE_VX] = _rotmat[0][0]*dt;
            A[STATE_Y][STATE_VX] = _rotmat[1][0]*dt;
            A[STATE_Z][STATE_VX] = _rotmat[2][0]*dt;

            A[STATE_X][STATE_VY] = _rotmat[0][1]*dt;
            A[STATE_Y][STATE_VY] = _rotmat[1][1]*dt;
            A[STATE_Z][STATE_VY] = _rotmat[2][1]*dt;

            A[STATE_X][STATE_VZ] = _rotmat[0][2]*dt;
            A[STATE_Y][STATE_VZ] = _rotmat[1][2]*dt;
            A[STATE_Z][STATE_VZ] = _rotmat[2][2]*dt;

            // position from attitude error
            A[STATE_X][STATE_D0] = (_state_vector[STATE_VY]*_rotmat[0][2] - 
                    _state_vector[STATE_VZ]*_rotmat[0][1])*dt;
            A[STATE_Y][STATE_D0] = (_state_vector[STATE_VY]*_rotmat[1][2] - 
                    _state_vector[STATE_VZ]*_rotmat[1][1])*dt;
            A[STATE_Z][STATE_D0] = (_state_vector[STATE_VY]*_rotmat[2][2] - 
                    _state_vector[STATE_VZ]*_rotmat[2][1])*dt;

            A[STATE_X][STATE_D1] = (- _state_vector[STATE_VX]*_rotmat[0][2] + 
                    _state_vector[STATE_VZ]*_rotmat[0][0])*dt;
            A[STATE_Y][STATE_D1] = (- _state_vector[STATE_VX]*_rotmat[1][2] + 
                    _state_vector[STATE_VZ]*_rotmat[1][0])*dt;
            A[STATE_Z][STATE_D1] = (- _state_vector[STATE_VX]*_rotmat[2][2] + 
                    _state_vector[STATE_VZ]*_rotmat[2][0])*dt;

            A[STATE_X][STATE_D2] = (_state_vector[STATE_VX]*_rotmat[0][1] - 
                    _state_vector[STATE_VY]*_rotmat[0][0])*dt;
            A[STATE_Y][STATE_D2] = (_state_vector[STATE_VX]*_rotmat[1][1] - 
                    _state_vector[STATE_VY]*_rotmat[1][0])*dt;
            A[STATE_Z][STATE_D2] = (_state_vector[STATE_VX]*_rotmat[2][1] - 
                    _state_vector[STATE_VY]*_rotmat[2][0])*dt;

            // body-frame velocity from body-frame velocity
            A[STATE_VX][STATE_VX] = 1; //drag negligible
            A[STATE_VY][STATE_VX] =-gyro->z*dt;
            A[STATE_VZ][STATE_VX] = gyro->y*dt;

            A[STATE_VX][STATE_VY] = gyro->z*dt;
            A[STATE_VY][STATE_VY] = 1; //drag negligible
            A[STATE_VZ][STATE_VY] =-gyro->x*dt;

            A[STATE_VX][STATE_VZ] =-gyro->y*dt;
            A[STATE_VY][STATE_VZ] = gyro->x*dt;
            A[STATE_VZ][STATE_VZ] = 1; //drag negligible

            // body-frame velocity from attitude error
            A[STATE_VX][STATE_D0] =  0;
            A[STATE_VY][STATE_D0] = -GRAVITY_MAGNITUDE*_rotmat[2][2]*dt;
            A[STATE_VZ][STATE_D0] =  GRAVITY_MAGNITUDE*_rotmat[2][1]*dt;

            A[STATE_VX][STATE_D1] =  GRAVITY_MAGNITUDE*_rotmat[2][2]*dt;
            A[STATE_VY][STATE_D1] =  0;
            A[STATE_VZ][STATE_D1] = -GRAVITY_MAGNITUDE*_rotmat[2][0]*dt;

            A[STATE_VX][STATE_D2] = -GRAVITY_MAGNITUDE*_rotmat[2][1]*dt;
            A[STATE_VY][STATE_D2] =  GRAVITY_MAGNITUDE*_rotmat[2][0]*dt;
            A[STATE_VZ][STATE_D2] =  0;


            // attitude error from attitude error
            /**
             * At first glance, it may not be clear where the next values come from,
             * since they do not appear directly in the dynamics. In this prediction
             * step, we skip the step of first updating attitude-error, and then
             * incorporating the
             * new error into the current attitude (which requires a rotation of the
             * attitude-error covariance). Instead, we directly update the body attitude,
             * however still need to rotate the covariance, which is what you see below.
             *
             * This comes from a second order approximation to:
             * Sigma_post = exps(-d) Sigma_pre exps(-d)'
             *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + 
             *             [[-d]]^2 / 2)'
             * where d is the attitude error expressed as Rodriges parameters, ie. d0 =
             * 1/2*gyro.x*dt under the assumption that d = [0,0,0] at the beginning of
             * each prediction step and that gyro.x is constant over the sampling period
             *
             * As derived in "Covariance Correction Step for Kalman Filtering with an 
             Attitude"
             * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
             */
            const float d0 = gyro->x*dt/2;
            const float d1 = gyro->y*dt/2;
            const float d2 = gyro->z*dt/2;

            A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
            A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
            A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

            A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
            A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
            A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

            A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
            A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
            A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

            updateCovariance(A);

            _ekf.updateCovariance(A);

            // Process noise is added after the return from the prediction step
            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust (hence is useless
            // to estimate body angle while flying)

            const float dt2 = dt * dt;

            if (isFlying) { // only acceleration in z direction

                // Use accelerometer and not commanded thrust, as this has
                // proper physical units
                const float zacc = accel->z;

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _state_vector[STATE_VX] * dt;
                const float dy = _state_vector[STATE_VY] * dt;
                const float dz = _state_vector[STATE_VZ] * dt + zacc * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _state_vector[STATE_X] += _rotmat[0][0] * dx + 
                    _rotmat[0][1] * dy + _rotmat[0][2] * dz;
                _state_vector[STATE_Y] += _rotmat[1][0] * dx + 
                    _rotmat[1][1] * dy + _rotmat[1][2] * dz;
                _state_vector[STATE_Z] += _rotmat[2][0] * dx + 
                    _rotmat[2][1] * dy + _rotmat[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                const float tmpSPX = _state_vector[STATE_VX];
                const float tmpSPY = _state_vector[STATE_VY];
                const float tmpSPZ = _state_vector[STATE_VZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _state_vector[STATE_VX] += dt * (gyro->z * tmpSPY - gyro->y *
                        tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][0]);
                _state_vector[STATE_VY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - 
                        GRAVITY_MAGNITUDE * _rotmat[2][1]);
                _state_vector[STATE_VZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _rotmat[2][2]);
            }
            else {
                // Acceleration can be in any direction, as measured by the
                // accelerometer. This occurs, eg. in freefall or while being carried.

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _state_vector[STATE_VX] * dt + accel->x * dt2 / 2.0f;
                const float dy = _state_vector[STATE_VY] * dt + accel->y * dt2 / 2.0f;
                const float dz = _state_vector[STATE_VZ] * dt + accel->z * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _state_vector[STATE_X] += _rotmat[0][0] * dx 
                    + _rotmat[0][1] * dy + _rotmat[0][2] * dz;
                _state_vector[STATE_Y] += _rotmat[1][0] * dx + 
                    _rotmat[1][1] * dy + _rotmat[1][2] * dz;
                _state_vector[STATE_Z] += _rotmat[2][0] * dx + 
                    _rotmat[2][1] * dy + _rotmat[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                const float tmpSPX = _state_vector[STATE_VX];
                const float tmpSPY = _state_vector[STATE_VY];
                const float tmpSPZ = _state_vector[STATE_VZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _state_vector[STATE_VX] += dt * (accel->x + gyro->z * tmpSPY -
                        gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][0]);
                _state_vector[STATE_VY] += dt * (accel->y - gyro->z * tmpSPX + gyro->x * 
                        tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][1]);
                _state_vector[STATE_VZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _rotmat[2][2]);
            }

            // attitude update (rotate by gyroscope), we do this in quaternions
            // this is the gyroscope angular velocity integrated over the sample period
            const float dtwx = dt*gyro->x;
            const float dtwy = dt*gyro->y;
            const float dtwz = dt*gyro->z;

            // compute the quaternion values in [w,x,y,z] order
            const float angle = device_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPSILON;
            const float ca = device_cos(angle/2.0f);
            const float sa = device_sin(angle/2.0f);
            const float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

            // rotate the vehicle's attitude by the delta quaternion vector computed above

            float tmpq0 = dq[0]*_quat[0] - dq[1]*_quat[1] - 
                dq[2]*_quat[2] - dq[3]*_quat[3];

            float tmpq1 = dq[1]*_quat[0] + dq[0]*_quat[1] + 
                dq[3]*_quat[2] - dq[2]*_quat[3];

            float tmpq2 = dq[2]*_quat[0] - dq[3]*_quat[1] + 
                dq[0]*_quat[2] + dq[1]*_quat[3];

            float tmpq3 = dq[3]*_quat[0] + dq[2]*_quat[1] - 
                dq[1]*_quat[2] + dq[0]*_quat[3];

            if (!isFlying) {

                const float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                tmpq0 = keep * tmpq0 + 
                    ROLLPITCH_ZERO_REVERSION * _initialQuaternion[0];
                tmpq1 = keep * tmpq1 + 
                    ROLLPITCH_ZERO_REVERSION * _initialQuaternion[1];
                tmpq2 = keep * tmpq2 + 
                    ROLLPITCH_ZERO_REVERSION * _initialQuaternion[2];
                tmpq3 = keep * tmpq3 + 
                    ROLLPITCH_ZERO_REVERSION * _initialQuaternion[3];
            }

            // normalize and store the result
            const float norm = device_sqrt(
                    tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPSILON;

            _quat[0] = tmpq0/norm; 
            _quat[1] = tmpq1/norm; 
            _quat[2] = tmpq2/norm; 
            _quat[3] = tmpq3/norm;

            _isUpdated = true;
            _lastPredictionMs = nowMs;
        }

        void addProcessNoise(const uint32_t nowMs) 
        {
            float dt = (nowMs - _lastProcessNoiseUpdateMs) / 1000.0f;

            if (dt > 0) {

                const float noise[STATE_DIM] = {
                    PROC_NOISE_ACCEL_XY*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_Z*dt*dt + PROC_NOISE_VEL*dt + PROC_NOISE_POS,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_XY*dt + PROC_NOISE_VEL,
                    PROC_NOISE_ACCEL_Z*dt + PROC_NOISE_VEL,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_ROLLPITCH * dt + PROC_NOISE_ATT,
                    MEAS_NOISE_GYRO_YAW * dt + PROC_NOISE_ATT
                };

                addCovarianceNoise(noise);
                enforceSymmetry();
                _lastProcessNoiseUpdateMs = nowMs;

                _ekf.addCovarianceNoise(noise, nowMs);
                _ekf.enforceSymmetry();
            }
        }

        void update(measurement_t & m, const uint32_t nowMs)
        {
            switch (m.type) {

                case MeasurementTypeTOF:
                    updateWithTof(&m.data.tof);
                    break;

                case MeasurementTypeFlow:
                    updateWithFlow(&m.data.flow);
                    break;

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

        void finalize(void)
        {
            // Matrix to rotate the attitude covariances once updated
            static float A[STATE_DIM][STATE_DIM];

            // Only finalize if data is updated
            if (! _isUpdated) {
                return;
            }

            // Incorporate the attitude error (Kalman filter state) with the attitude
            const float v0 = _state_vector[STATE_D0];
            const float v1 = _state_vector[STATE_D1];
            const float v2 = _state_vector[STATE_D2];

            // Move attitude error into attitude if any of the angle errors are
            // large enough
            if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) >
                        0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 &&
                            fabsf(v2) < 10)) {

                const float angle = device_sqrt(v0*v0 + v1*v1 + v2*v2) + EPSILON;
                const float ca = device_cos(angle / 2.0f);
                const float sa = device_sin(angle / 2.0f);
                const float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

                // Rotate the vehicle's attitude by the delta quaternion vector
                // computed above
                const float tmpq0 = dq[0] * _quat[0] - dq[1] * _quat[1] - 
                    dq[2] * _quat[2] - dq[3] * _quat[3];
                const float tmpq1 = dq[1] * _quat[0] + dq[0] * _quat[1] + 
                    dq[3] * _quat[2] - dq[2] * _quat[3];
                const float tmpq2 = dq[2] * _quat[0] - dq[3] * _quat[1] + 
                    dq[0] * _quat[2] + dq[1] * _quat[3];
                const float tmpq3 = dq[3] * _quat[0] + dq[2] * _quat[1] - 
                    dq[1] * _quat[2] + dq[0] * _quat[3];

                // normalize and store the result
                float norm = device_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                        tmpq3 * tmpq3) + EPSILON;
                _quat[0] = tmpq0 / norm;
                _quat[1] = tmpq1 / norm;
                _quat[2] = tmpq2 / norm;
                _quat[3] = tmpq3 / norm;

                /** Rotate the covariance, since we've rotated the body
                 *
                 * This comes from a second order approximation to:
                 * Sigma_post = exps(-d) Sigma_pre exps(-d)'
                 *            ~ (I + [[-d]] + [[-d]]^2 / 2) 
                 Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
                 * where d is the attitude error expressed as Rodriges parameters, ie. 
                 d = tan(|v|/2)*v/|v|
                 *
                 * As derived in "Covariance Correction Step for Kalman Filtering with an 
                 Attitude"
                 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
                 */

                // the attitude error vector (v0,v1,v2) is small,
                // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
                const float d0 = v0/2; 
                const float d1 = v1/2; 
                const float d2 = v2/2;

                A[STATE_X][STATE_X] = 1;
                A[STATE_Y][STATE_Y] = 1;
                A[STATE_Z][STATE_Z] = 1;

                A[STATE_VX][STATE_VX] = 1;
                A[STATE_VY][STATE_VY] = 1;
                A[STATE_VZ][STATE_VZ] = 1;

                A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
                A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

                A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
                A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

                A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
                A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
                A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

                _ekf.updateCovariance(A);
            }

            // Convert the new attitude to a rotation matrix, such that we can
            // rotate body-frame velocity and acc

            _rotmat[0][0] = _quat[0] * _quat[0] + 
                _quat[1] * _quat[1] - _quat[2] * 
                _quat[2] - _quat[3] * _quat[3];

            _rotmat[0][1] = 2 * _quat[1] * _quat[2] - 
                2 * _quat[0] * _quat[3];

            _rotmat[0][2] = 2 * _quat[1] * _quat[3] + 
                2 * _quat[0] * _quat[2];

            _rotmat[1][0] = 2 * _quat[1] * _quat[2] + 
                2 * _quat[0] * _quat[3];

            _rotmat[1][1] = _quat[0] * _quat[0] - 
                _quat[1] * _quat[1] + _quat[2] * 
                _quat[2] - _quat[3] * _quat[3];

            _rotmat[1][2] = 2 * _quat[2] * _quat[3] - 
                2 * _quat[0] * _quat[1];

            _rotmat[2][0] = 2 * _quat[1] * _quat[3] - 
                2 * _quat[0] * _quat[2];

            _rotmat[2][1] = 2 * _quat[2] * _quat[3] + 
                2 * _quat[0] * _quat[1];

            _rotmat[2][2] = _quat[0] * _quat[0] - 
                _quat[1] * _quat[1] - _quat[2] * 
                _quat[2] + _quat[3] * _quat[3];

            // reset the attitude error
            _state_vector[STATE_D0] = 0;
            _state_vector[STATE_D1] = 0;
            _state_vector[STATE_D2] = 0;

            enforceSymmetry();

            _ekf.enforceSymmetry();

            _isUpdated = false;
        }


        bool isStateWithinBounds(void) 
        {
            for (int i = 0; i < 3; i++) {

                if (MAX_VELOCITY > 0.0f) {
                    if (_state_vector[STATE_VX + i] > MAX_VELOCITY) {
                        return false;
                    } else if (_state_vector[STATE_VX + i] < -MAX_VELOCITY) {
                        return false;
                    }
                }
            }

            return true;
        }

        void getVehicleState(vehicleState_t & state)
        {
            state.x = _state_vector[STATE_X];

            state.dx = _rotmat[0][0]*_state_vector[STATE_VX] + 
                _rotmat[0][1]*_state_vector[STATE_VY] + 
                _rotmat[0][2]*_state_vector[STATE_VZ];

            state.y = _state_vector[STATE_Y];

            // Negate for rightward positive
            state.dy = -(_rotmat[1][0]*_state_vector[STATE_VX] + 
                    _rotmat[1][1]*_state_vector[STATE_VY] + 
                    _rotmat[1][2]*_state_vector[STATE_VZ]);

            state.z = _state_vector[STATE_Z];

            state.dz = _rotmat[2][0]*_state_vector[STATE_VX] + 
                _rotmat[2][1]*_state_vector[STATE_VY] + 
                _rotmat[2][2]*_state_vector[STATE_VZ];

            state.phi = RADIANS_TO_DEGREES *
                atan2f(2*(_quat[2]*_quat[3]+_quat[0]*
                            _quat[1]) ,
                        _quat[0]*_quat[0] -
                        _quat[1]*_quat[1] -
                        _quat[2]*_quat[2] +
                        _quat[3]*_quat[3]);

            state.theta = RADIANS_TO_DEGREES * 
                asinf(-2*(_quat[1]*_quat[3] -
                            _quat[0]*_quat[2]));

            state.psi = -RADIANS_TO_DEGREES *   // negate for nose-right positive
                atan2f(2*(_quat[1]*_quat[2]+_quat[0]*
                            _quat[3])
                        , _quat[0]*_quat[0] +
                        _quat[1]*_quat[1] -
                        _quat[2]*_quat[2] -
                        _quat[3]*_quat[3]);
        }

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

        static constexpr float GRAVITY_MAGNITUDE = 9.81;

        static constexpr float DEGREES_TO_RADIANS = PI / 180.0f;
        static constexpr float RADIANS_TO_DEGREES = 180.0f / PI;

        //We do get the measurements in 10x the motion pixels (experimentally measured)
        static constexpr float FLOW_RESOLUTION = 0.1;

        // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
        static constexpr float MAX_COVARIANCE = 100;
        static constexpr float MIN_COVARIANCE = 1e-6;

        // The bounds on states, these shouldn't be hit...
        static constexpr float MAX_VELOCITY = 10; //meters per second

        // Small number epsilon, to prevent dividing by zero
        static constexpr float EPSILON = 1e-6f;

        // the reversion of pitch and roll to zero
        static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

        typedef struct {
            Axis3f sum;
            uint32_t count;
            float conversionFactor;

            Axis3f subSample;
        } Axis3fSubSampler_t;

        // Quaternion used for initial orientation [w,x,y,z]
        float _initialQuaternion[4];

        Axis3f _accLatest;
        Axis3f _gyroLatest;

        Axis3fSubSampler_t _accSubSampler;
        Axis3fSubSampler_t _gyroSubSampler;

        OutlierFilterTdoa _outlierFilterTdoa;

        float _predictedNX;
        float _predictedNY;

        float _measuredNX;
        float _measuredNY;

        /**
         * Vehicle State
         *
         * The internally-estimated state is:
         * - X, Y, Z: the vehicle's position in the global frame
         * - PX, PY, PZ: the vehicle's velocity in its body frame
         * - D0, D1, D2: attitude error
         *
         * For more information, refer to the paper
         */
        float _state_vector[STATE_DIM];

        // The vehicle's attitude as a rotation matrix (used by the prediction,
        // updated by the finalization)
        float _rotmat[3][3];

        // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
        // to allow easy normalization (in comparison to a rotation matrix),
        // while also being robust against singularities (in comparison to euler angles)
        float _quat[4];

        static void axis3fSubSamplerInit(Axis3fSubSampler_t* subSampler, const
                float conversionFactor) { memset(subSampler, 0,
                    sizeof(Axis3fSubSampler_t));
                subSampler->conversionFactor = conversionFactor;
        }

        static void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* subSampler,
                const Axis3f* sample) {
            subSampler->sum.x += sample->x;
            subSampler->sum.y += sample->y;
            subSampler->sum.z += sample->z;

            subSampler->count++;
        }

        static Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* subSampler) 
        {
            if (subSampler->count > 0) {
                subSampler->subSample.x = 
                    subSampler->sum.x * subSampler->conversionFactor / subSampler->count;
                subSampler->subSample.y = 
                    subSampler->sum.y * subSampler->conversionFactor / subSampler->count;
                subSampler->subSample.z = 
                    subSampler->sum.z * subSampler->conversionFactor / subSampler->count;

                // Reset
                subSampler->count = 0;
                subSampler->sum = (Axis3f){.axis={0}};
            }

            return &subSampler->subSample;
        }

        void updateWithFlow(const flowMeasurement_t *flow) 
        {
            const Axis3f *gyro = &_gyroLatest;

            // Inclusion of flow measurements in the EKF done by two scalar updates

            // ~~~ Camera constants ~~~
            // The angle of aperture is guessed from the raw data register and
            // thankfully look to be symmetric

            // [pixels] (same in x and y)
            float Npix = 35.0;                      

            //float thetapix = DEGREES_TO_RADIANS * 4.0f;
            // [rad]    (same in x and y)
            // 2*sin(42/2); 42degree is the agnle of aperture, here we computed the
            // corresponding ground length
            float thetapix = 0.71674f;

            //~~~ Body rates ~~~
            // TODO check if this is feasible or if some filtering has to be done
            float omegax_b = gyro->x * DEGREES_TO_RADIANS;
            float omegay_b = gyro->y * DEGREES_TO_RADIANS;

            // ~~~ Moves the body velocity into the global coordinate system ~~~
            // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
            //
            // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
            // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
            //
            // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
            // _G and _B refer to the global/body coordinate systems.

            // Modification 1
            //dx_g = R[0][0] * S[STATE_VX] + R[0][1] * S[STATE_VY] + R[0][2] * 
            //  S[STATE_VZ];
            //dy_g = R[1][0] * S[STATE_VX] + R[1][1] * S[STATE_VY] + R[1][2] * 
            //  S[STATE_VZ];


            float dx_g = _state_vector[STATE_VX];
            float dy_g = _state_vector[STATE_VY];
            float z_g = 0.0;
            // Saturate elevation in prediction and correction to avoid singularities
            if ( _state_vector[STATE_Z] < 0.1f ) {
                z_g = 0.1;
            } else {
                z_g = _state_vector[STATE_Z];
            }

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            float hx[STATE_DIM] = {};
            _predictedNX = (flow->dt * Npix / thetapix ) * 
                ((dx_g * _rotmat[2][2] / z_g) - omegay_b);
            _measuredNX = flow->dpixelx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            hx[STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_rotmat[2][2] * dx_g) / (-z_g * z_g));
            hx[STATE_VX] = (Npix * flow->dt / thetapix) * 
                (_rotmat[2][2] / z_g);

            //First update
            matrix_t Hx = {1, STATE_DIM, hx};
            updateWithScalar(Hx, (_measuredNX-_predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION);

            _ekf.updateWithScalar(Hx, (_measuredNX-_predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION);

            // ~~~ Y velocity prediction and update ~~~
            float hy[STATE_DIM] = {};
            _predictedNY = (flow->dt * Npix / thetapix ) * 
                ((dy_g * _rotmat[2][2] / z_g) + omegax_b);
            _measuredNY = flow->dpixely*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            hy[STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_rotmat[2][2] * dy_g) / (-z_g * z_g));
            hy[STATE_VY] = (Npix * flow->dt / thetapix) * (_rotmat[2][2] / z_g);

            // Second update
            matrix_t Hy = {1, STATE_DIM, hy};
            updateWithScalar(Hy, (_measuredNY-_predictedNY),
                    flow->stdDevY*FLOW_RESOLUTION);

            _ekf.updateWithScalar(Hy, (_measuredNY-_predictedNY),
                    flow->stdDevY*FLOW_RESOLUTION);
        }

        void updateWithTof(tofMeasurement_t *tof)
        {
            // Updates the filter with a measured distance in the zb direction using the
            float h[STATE_DIM] = {};
            matrix_t H = {1, STATE_DIM, h};

            // Only update the filter if the measurement is reliable 
            // (\hat{h} -> infty when R[2][2] -> 0)
            if (fabs(_rotmat[2][2]) > 0.1f && _rotmat[2][2] > 0) {
                float angle = 
                    fabsf(acosf(_rotmat[2][2])) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f);
                if (angle < 0.0f) {
                    angle = 0.0f;
                }
                float predictedDistance = _state_vector[STATE_Z] / cosf(angle);
                float measuredDistance = tof->distance; // [m]


                // The sensor model (Pg.95-96,
                // https://lup.lub.lu.se/student-papers/search/publication/8905295)
                //
                // h = z/((R*z_b).z_b) = z/cos(alpha)
                //
                // Here,
                // h (Measured variable)[m] = Distance given by TOF sensor. This is the 
                // closest point from any surface to the sensor in the measurement cone
                // z (Estimated variable)[m] = THe actual elevation of the crazyflie
                // z_b = Basis vector in z direction of body coordinate system
                // R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be 
                // stationary
                // alpha = angle between [line made by measured point <---> sensor] 
                // and [the intertial z-axis] 

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                h[STATE_Z] = 1 / cosf(angle); 

                updateWithScalar(H, measuredDistance-predictedDistance, tof->stdDev);
            }
        }

        void updateWithAccel(measurement_t & m)
        {
            axis3fSubSamplerAccumulate(&_accSubSampler, &m.data.acceleration.acc);
            _accLatest = m.data.acceleration.acc;
        }

        void updateWithGyro(measurement_t & m)
        {
            axis3fSubSamplerAccumulate(&_gyroSubSampler, &m.data.gyroscope.gyro);
            _gyroLatest = m.data.gyroscope.gyro;
        }

        // Generic EKF stuff //////////////////////////////////////////////////

        EKF _ekf;

        // The covariance matrix
        __attribute__((aligned(4))) float _Pmatrix[STATE_DIM][STATE_DIM];
        matrix_t _Pmatrix_m;

        // Tracks whether an update to the state has been made, and the state
        // therefore requires finalization
        bool _isUpdated;

        uint32_t _lastPredictionMs;
        uint32_t _lastProcessNoiseUpdateMs;

        void updateCovariance(const float A[STATE_DIM][STATE_DIM])
        {
            static __attribute__((aligned(4))) matrix_t Am = { 
                STATE_DIM, STATE_DIM, (float *)A
            };

            static float tmpNN1d[STATE_DIM * STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN1m = { 
                STATE_DIM, STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[STATE_DIM * STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN2m = { 
                STATE_DIM, STATE_DIM, tmpNN2d
            };

            device_mat_mult(&Am, &_Pmatrix_m, &tmpNN1m); // A P
            device_mat_trans(&Am, &tmpNN2m); // A'
            device_mat_mult(&tmpNN1m, &tmpNN2m, &_Pmatrix_m); // A P A'

        }

        void addCovarianceNoise(const float * noise)
        {
            for (uint8_t k=0; k<STATE_DIM; ++k) {
                _Pmatrix[k][k] += noise[k] * noise[k];
            }
        }

        void updateWithScalar(
                matrix_t & Hm, const float error, const float stdMeasNoise)
        {
            // The Kalman gain as a column vector
            static float G[STATE_DIM];
            static matrix_t Gm = {STATE_DIM, 1, (float *)G};

            // Temporary matrices for the covariance updates
            static float tmpNN1d[STATE_DIM * STATE_DIM];
            static matrix_t tmpNN1m = {
                STATE_DIM, STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[STATE_DIM * STATE_DIM];
            static matrix_t tmpNN2m = {
                STATE_DIM, STATE_DIM, tmpNN2d
            };

            static float tmpNN3d[STATE_DIM * STATE_DIM];
            static matrix_t tmpNN3m = {
                STATE_DIM, STATE_DIM, tmpNN3d
            };

            static float HTd[STATE_DIM * 1];
            static matrix_t HTm = {STATE_DIM, 1, HTd};

            static float PHTd[STATE_DIM * 1];
            static matrix_t PHTm = {STATE_DIM, 1, PHTd};

            // ====== INNOVATION COVARIANCE ======

            device_mat_trans(&Hm, &HTm);
            device_mat_mult(&_Pmatrix_m, &HTm, &PHTm); // PH'
            float R = stdMeasNoise*stdMeasNoise;
            float HPHR = R; // HPH' + R
            for (int i=0; i<STATE_DIM; i++) { 
                // Add the element of HPH' to the above
                // this obviously only works if the update is scalar (as in this function)
                HPHR += Hm.pData[i]*PHTd[i]; 
            }

            // ====== MEASUREMENT UPDATE ======
            // Calculate the Kalman gain and perform the state update
            for (int i=0; i<STATE_DIM; i++) {
                G[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                _state_vector[i] = _state_vector[i] + G[i] * error; // state update
            }

            // ====== COVARIANCE UPDATE ======
            device_mat_mult(&Gm, &Hm, &tmpNN1m); // KH
            for (int i=0; i<STATE_DIM; i++) { 
                tmpNN1d[STATE_DIM*i+i] -= 1; 
            } // GH - I
            device_mat_trans(&tmpNN1m, &tmpNN2m); // (GH - I)'
            device_mat_mult(&tmpNN1m, &_Pmatrix_m, &tmpNN3m); // (GH - I)*P
            device_mat_mult(&tmpNN3m, &tmpNN2m, &_Pmatrix_m); // (GH - I)*P*(GH - I)'

            // add the measurement variance and ensure boundedness and symmetry
            // TODO: Why would it hit these bounds? Needs to be investigated.
            for (int i=0; i<STATE_DIM; i++) {
                for (int j=i; j<STATE_DIM; j++) {
                    float v = G[i] * R * G[j];

                    // add measurement noise
                    float p = 0.5f*_Pmatrix[i][j] + 0.5f*_Pmatrix[j][i] + v; 
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = MIN_COVARIANCE;
                    } else {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = p;
                    }
                }
            }

            _isUpdated = true;
        }

        void enforceSymmetry()
        {
            for (int i=0; i<STATE_DIM; i++) {
                for (int j=i; j<STATE_DIM; j++) {
                    float p = 0.5f*_Pmatrix[i][j] + 0.5f*_Pmatrix[j][i];
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = MIN_COVARIANCE;
                    } else {
                        _Pmatrix[i][j] = _Pmatrix[j][i] = p;
                    }
                }
            }

        }

        // Generic math stuff //////////////////////////////////////////////////

        static void device_mat_trans(const matrix_t * pSrc, matrix_t * pDst); 

        static void device_mat_mult(
                const matrix_t * pSrcA, const matrix_t * pSrcB,
                matrix_t * pDst);

        static float device_cos(const float x);

        static float device_sin(const float x);

        static float device_sqrt(const float32_t in);
};
