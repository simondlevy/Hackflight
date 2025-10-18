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
 * "Fusing ultra-wideband range measurements with accelerometers and rate
 * gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:

 @INPROCEEDINGS{MuellerHamerUWB2015,
 author = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
 title  = {Fusing ultra-wideband range measurements with accelerometers and rate 
 gyroscopes for quadrocopter state estimation},
 booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
 year   = {2015},
 month  = {May},
 pages  = {1730-1736},
 doi    = {10.1109/ICRA.2015.7139421},
 ISSN   = {1050-4729}}

 @ARTICLE{MuellerCovariance2016,
 author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
 title={Covariance Correction Step for Kalman Filtering with an Attitude},
 journal={Journal of Guidance, Control, and Dynamics},
 pages={1--7},
 year={2016},
 publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from 
 OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 * 2023.09.10, Simon D. Levy: Made a header-only C++ class
 */

#pragma once

#include <math3d.h>
#include <outlierFilterTdoa.hpp>
#include <datatypes.h>
#include <matrix_typedef.h>

class KalmanFilter { 

    public:

        typedef enum {
            MeasurementTypeTDOA,
            MeasurementTypePosition,
            MeasurementTypePose,
            MeasurementTypeDistance,
            MeasurementTypeTOF,
            MeasurementTypeAbsoluteHeight,
            MeasurementTypeFlow,
            MeasurementTypeYawError,
            MeasurementTypeSweepAngle,
            MeasurementTypeGyroscope,
            MeasurementTypeAcceleration,
            MeasurementTypeBarometer,
        } MeasurementType;

        typedef struct
        {
            MeasurementType type;
            union
            {
                tdoaMeasurement_t tdoa;
                positionMeasurement_t position;
                poseMeasurement_t pose;
                distanceMeasurement_t distance;
                tofMeasurement_t tof;
                heightMeasurement_t height;
                flowMeasurement_t flow;
                yawErrorMeasurement_t yawError;
                sweepAngleMeasurement_t sweepAngle;
                gyroscopeMeasurement_t gyroscope;
                accelerationMeasurement_t acceleration;
                barometerMeasurement_t barometer;
            } data;
        } measurement_t;


        // Indexes to access the quad's state, stored as a column vector
        typedef enum
        {
            KC_STATE_X,
            KC_STATE_Y,
            KC_STATE_Z,
            KC_STATE_PX,
            KC_STATE_PY,
            KC_STATE_PZ,
            KC_STATE_D0,
            KC_STATE_D1,
            KC_STATE_D2,
            KC_STATE_DIM

        } kalmanCoreStateIdx_t;


        // The data used by the kalman algo implementation.
        typedef struct {
            /**
             * Vehicle State
             *
             * The internally-estimated state is:
             * - X, Y, Z: the quad's position in the global frame
             * - PX, PY, PZ: the quad's velocity in its body frame
             * - D0, D1, D2: attitude error
             *
             * For more information, refer to the paper
             */
            float S[KC_STATE_DIM];

            // The quad's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            float q[4];

            // The quad's attitude as a rotation matrix (used by the prediction,
            // updated by the finalization)
            float R[3][3];

            // The covariance matrix
            __attribute__((aligned(4))) float P[KC_STATE_DIM][KC_STATE_DIM];
            matrix_t Pm;

            float baroReferenceHeight;

            // Quaternion used for initial orientation [w,x,y,z]
            float initialQuaternion[4];

            // Tracks whether an update to the state has been made, and the state
            // therefore requires finalization
            bool isUpdated;

            uint32_t lastPredictionMs;
            uint32_t lastProcessNoiseUpdateMs;

        } kalmanCoreData_t;

        void setDefaultParams(void)
        {
            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            _params.stdDevInitialPosition_xy = 100;
            _params.stdDevInitialPosition_z = 1;
            _params.stdDevInitialVelocity = 0.01;
            _params.stdDevInitialAttitude_rollpitch = 0.01;
            _params.stdDevInitialAttitude_yaw = 0.01;

            _params.procNoiseAcc_xy = 0.5f;
            _params.procNoiseAcc_z = 1.0f;
            _params.procNoiseVel = 0;
            _params.procNoisePos = 0;
            _params.procNoiseAtt = 0;
            _params.measNoiseBaro = 2.0f;           // meters
            _params.measNoiseGyro_rollpitch = 0.1f; // radians per second
            _params.measNoiseGyro_yaw = 0.1f;       // radians per second

            _params.initialX = 0.0;
            _params.initialY = 0.0;
            _params.initialZ = 0.0;

            // Initial yaw of the Crazyflie in radians.
            // 0 --- facing positive X
            // PI / 2 --- facing positive Y
            // PI --- facing negative X
            // 3 * PI / 2 --- facing negative Y
            _params.initialYaw = 0.0;
        }

        void init(const uint32_t nowMs)
        {
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY_MAGNITUDE);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            _outlierFilterTdoa.reset();

            // Reset all data to 0 (like upon system reset)
            memset(&_kalmanData, 0, sizeof(kalmanCoreData_t));

            _kalmanData.S[KC_STATE_X] = _params.initialX;
            _kalmanData.S[KC_STATE_Y] = _params.initialY;
            _kalmanData.S[KC_STATE_Z] = _params.initialZ;

            // reset the attitude quaternion
            _kalmanData.initialQuaternion[0] = device_cos(_params.initialYaw / 2);
            _kalmanData.initialQuaternion[1] = 0.0;
            _kalmanData.initialQuaternion[2] = 0.0;
            _kalmanData.initialQuaternion[3] = device_sin(_params.initialYaw / 2);

            for (int i = 0; i < 4; i++) { 
                _kalmanData.q[i] = _kalmanData.initialQuaternion[i]; 
            }

            // then set the initial rotation matrix to the identity. This only affects
            // the first prediction step, since in the finalization, after shifting
            // attitude errors into the attitude state, the rotation matrix is updated.
            for(int i=0; i<3; i++) { 
                for(int j=0; j<3; j++) { 
                    _kalmanData.R[i][j] = i==j ? 1 : 0; 
                }
            }

            for (int i=0; i< KC_STATE_DIM; i++) {

                for (int j=0; j < KC_STATE_DIM; j++) {

                    // set covariances to zero (diagonals will be changed from
                    // zero in the next section)
                    _kalmanData.P[i][j] = 0; 
                }
            }

            // initialize state variances
            _kalmanData.P[KC_STATE_X][KC_STATE_X] = 
                powf(_params.stdDevInitialPosition_xy, 2);
            _kalmanData.P[KC_STATE_Y][KC_STATE_Y] = 
                powf(_params.stdDevInitialPosition_xy, 2);
            _kalmanData.P[KC_STATE_Z][KC_STATE_Z] = 
                powf(_params.stdDevInitialPosition_z, 2);

            _kalmanData.P[KC_STATE_PX][KC_STATE_PX] = 
                powf(_params.stdDevInitialVelocity, 2);
            _kalmanData.P[KC_STATE_PY][KC_STATE_PY] = 
                powf(_params.stdDevInitialVelocity, 2);
            _kalmanData.P[KC_STATE_PZ][KC_STATE_PZ] = 
                powf(_params.stdDevInitialVelocity, 2);

            _kalmanData.P[KC_STATE_D0][KC_STATE_D0] = 
                powf(_params.stdDevInitialAttitude_rollpitch, 2);
            _kalmanData.P[KC_STATE_D1][KC_STATE_D1] = 
                powf(_params.stdDevInitialAttitude_rollpitch, 2);
            _kalmanData.P[KC_STATE_D2][KC_STATE_D2] = 
                powf(_params.stdDevInitialAttitude_yaw, 2);

            _kalmanData.Pm.numRows = KC_STATE_DIM;
            _kalmanData.Pm.numCols = KC_STATE_DIM;
            _kalmanData.Pm.pData = (float*)_kalmanData.P;

            _kalmanData.baroReferenceHeight = 0.0;

            _kalmanData.isUpdated = false;
            _kalmanData.lastPredictionMs = nowMs;
            _kalmanData.lastProcessNoiseUpdateMs = nowMs;
        }

        void predictDt(Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
        {
            // The linearized update matrix
            static float A[KC_STATE_DIM][KC_STATE_DIM];
            static __attribute__((aligned(4))) matrix_t Am = { 
                KC_STATE_DIM, KC_STATE_DIM, (float *)A
            }; // linearized dynamics for covariance update;

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN1m = { 
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN2m = { 
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            };

            predictDt(A, &Am, &tmpNN1m, &tmpNN2m, acc, gyro, dt, quadIsFlying);
        }

        void finalize(
                float  A[KC_STATE_DIM][KC_STATE_DIM],
                matrix_t * Am,
                matrix_t * tmpNN1m,
                matrix_t * tmpNN2m)
        {
            // Only finalize if data is updated
            if (! _kalmanData.isUpdated) {
                return;
            }

            // Incorporate the attitude error (Kalman filter state) with the attitude
            float v0 = _kalmanData.S[KC_STATE_D0];
            float v1 = _kalmanData.S[KC_STATE_D1];
            float v2 = _kalmanData.S[KC_STATE_D2];

            // Move attitude error into attitude if any of the angle errors are
            // large enough
            if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) >
                        0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 &&
                            fabsf(v2) < 10)) {
                float angle = device_sqrt(v0*v0 + v1*v1 + v2*v2) + EPSILON;
                float ca = device_cos(angle / 2.0f);
                float sa = device_sin(angle / 2.0f);
                float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

                // Rotate the quad's attitude by the delta quaternion vector
                // computed above
                float tmpq0 = dq[0] * _kalmanData.q[0] - dq[1] * _kalmanData.q[1] - 
                    dq[2] * _kalmanData.q[2] - dq[3] * _kalmanData.q[3];
                float tmpq1 = dq[1] * _kalmanData.q[0] + dq[0] * _kalmanData.q[1] + 
                    dq[3] * _kalmanData.q[2] - dq[2] * _kalmanData.q[3];
                float tmpq2 = dq[2] * _kalmanData.q[0] - dq[3] * _kalmanData.q[1] + 
                    dq[0] * _kalmanData.q[2] + dq[1] * _kalmanData.q[3];
                float tmpq3 = dq[3] * _kalmanData.q[0] + dq[2] * _kalmanData.q[1] - 
                    dq[1] * _kalmanData.q[2] + dq[0] * _kalmanData.q[3];

                // normalize and store the result
                float norm = device_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                        tmpq3 * tmpq3) + EPSILON;
                _kalmanData.q[0] = tmpq0 / norm;
                _kalmanData.q[1] = tmpq1 / norm;
                _kalmanData.q[2] = tmpq2 / norm;
                _kalmanData.q[3] = tmpq3 / norm;

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
                float d0 = v0/2; 
                float d1 = v1/2; 
                float d2 = v2/2;

                A[KC_STATE_X][KC_STATE_X] = 1;
                A[KC_STATE_Y][KC_STATE_Y] = 1;
                A[KC_STATE_Z][KC_STATE_Z] = 1;

                A[KC_STATE_PX][KC_STATE_PX] = 1;
                A[KC_STATE_PY][KC_STATE_PY] = 1;
                A[KC_STATE_PZ][KC_STATE_PZ] = 1;

                A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
                A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
                A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

                A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
                A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
                A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

                A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
                A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
                A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

                device_mat_trans(Am, tmpNN1m); // A'
                device_mat_mult(Am, &_kalmanData.Pm, tmpNN2m); // AP
                device_mat_mult(tmpNN2m, tmpNN1m, &_kalmanData.Pm); //APA'
            }

            // Convert the new attitude to a rotation matrix, such that we can
            // rotate body-frame velocity and acc

            _kalmanData.R[0][0] = _kalmanData.q[0] * _kalmanData.q[0] + 
                _kalmanData.q[1] * _kalmanData.q[1] - _kalmanData.q[2] * 
                _kalmanData.q[2] - _kalmanData.q[3] * _kalmanData.q[3];

            _kalmanData.R[0][1] = 2 * _kalmanData.q[1] * _kalmanData.q[2] - 
                2 * _kalmanData.q[0] * _kalmanData.q[3];

            _kalmanData.R[0][2] = 2 * _kalmanData.q[1] * _kalmanData.q[3] + 
                2 * _kalmanData.q[0] * _kalmanData.q[2];

            _kalmanData.R[1][0] = 2 * _kalmanData.q[1] * _kalmanData.q[2] + 
                2 * _kalmanData.q[0] * _kalmanData.q[3];

            _kalmanData.R[1][1] = _kalmanData.q[0] * _kalmanData.q[0] - 
                _kalmanData.q[1] * _kalmanData.q[1] + _kalmanData.q[2] * 
                _kalmanData.q[2] - _kalmanData.q[3] * _kalmanData.q[3];

            _kalmanData.R[1][2] = 2 * _kalmanData.q[2] * _kalmanData.q[3] - 
                2 * _kalmanData.q[0] * _kalmanData.q[1];

            _kalmanData.R[2][0] = 2 * _kalmanData.q[1] * _kalmanData.q[3] - 
                2 * _kalmanData.q[0] * _kalmanData.q[2];

            _kalmanData.R[2][1] = 2 * _kalmanData.q[2] * _kalmanData.q[3] + 
                2 * _kalmanData.q[0] * _kalmanData.q[1];

            _kalmanData.R[2][2] = _kalmanData.q[0] * _kalmanData.q[0] - 
                _kalmanData.q[1] * _kalmanData.q[1] - _kalmanData.q[2] * 
                _kalmanData.q[2] + _kalmanData.q[3] * _kalmanData.q[3];

            // reset the attitude error
            _kalmanData.S[KC_STATE_D0] = 0;
            _kalmanData.S[KC_STATE_D1] = 0;
            _kalmanData.S[KC_STATE_D2] = 0;

            // enforce symmetry of the covariance matrix, and ensure the values
            // stay bounded
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float p = 0.5f*_kalmanData.P[i][j] + 0.5f*_kalmanData.P[j][i];
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MIN_COVARIANCE;
                    } else {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = p;
                    }
                }
            }

            _kalmanData.isUpdated = false;
        }

        void predict(const uint32_t nowMs, bool quadIsFlying) 
        {
            axis3fSubSamplerFinalize(&_accSubSampler);
            axis3fSubSamplerFinalize(&_gyroSubSampler);

            float dt = (nowMs - _kalmanData.lastPredictionMs) / 1000.0f;

            predictDt(&_accSubSampler.subSample, &_gyroSubSampler.subSample, dt,
                    quadIsFlying);

            _kalmanData.lastPredictionMs = nowMs;
        }

        void addProcessNoise(const uint32_t nowMs) 
        {
            float dt = (nowMs - _kalmanData.lastProcessNoiseUpdateMs) / 1000.0f;

            if (dt > 0.0f) {
                addProcessNoiseDt(dt);
                _kalmanData.lastProcessNoiseUpdateMs = nowMs;
            }
        }

        void update(measurement_t & m, const uint32_t nowMs)
        {
            switch (m.type) {

                case MeasurementTypeTDOA:
                    updateWithTdoa(&m.data.tdoa, nowMs);
                    break;
                case MeasurementTypePosition:
                    updateWithPosition(&m.data.position);
                    break;
                case MeasurementTypePose:
                    updateWithPose(&m.data.pose);
                    break;
                case MeasurementTypeDistance:
                    updateWithDistance(&m.data.distance);
                    break;
                case MeasurementTypeTOF:
                    updateWithTof(&m.data.tof);
                    break;
                case MeasurementTypeAbsoluteHeight:
                    updateWithAbsoluteHeight(&m.data.height);
                    break;
                case MeasurementTypeFlow:
                    updateWithFlow(&m.data.flow);
                    break;
                case MeasurementTypeYawError:
                    updateWithYawError(&m.data.yawError);
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
            static float A[KC_STATE_DIM][KC_STATE_DIM];
            static matrix_t Am = {
                KC_STATE_DIM, KC_STATE_DIM, (float *)A
            };

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static matrix_t tmpNN1m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static matrix_t tmpNN2m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            }; 
            return finalize(A, &Am, &tmpNN1m, &tmpNN2m);
        }

        bool isStateWithinBounds(void) 
        {
            for (int i = 0; i < 3; i++) {

                if (MAX_POSITITON > 0.0f) {

                    if (_kalmanData.S[KC_STATE_X + i] > MAX_POSITITON) {
                        return false;
                    } else if (_kalmanData.S[KC_STATE_X + i] < -MAX_POSITITON) {
                        return false;
                    }
                }

                if (MAX_VELOCITY > 0.0f) {
                    if (_kalmanData.S[KC_STATE_PX + i] > MAX_VELOCITY) {
                        return false;
                    } else if (_kalmanData.S[KC_STATE_PX + i] < -MAX_VELOCITY) {
                        return false;
                    }
                }
            }

            return true;
        }

        void updateWithQuaternion(const quaternion_t & quat)
        {
            _kalmanData.q[0] = quat.w;
            _kalmanData.q[1] = quat.x;
            _kalmanData.q[2] = quat.y;
            _kalmanData.q[3] = quat.z;
        }

        void getVehicleState(vehicleState_t & state)
        {
            state.x = _kalmanData.S[KC_STATE_X];

            state.dx = _kalmanData.R[0][0]*_kalmanData.S[KC_STATE_PX] + 
                    _kalmanData.R[0][1]*_kalmanData.S[KC_STATE_PY] + 
                    _kalmanData.R[0][2]*_kalmanData.S[KC_STATE_PZ];

            state.y = _kalmanData.S[KC_STATE_Y];

            // Negate for rightward positive
            state.dy = -(_kalmanData.R[1][0]*_kalmanData.S[KC_STATE_PX] + 
                        _kalmanData.R[1][1]*_kalmanData.S[KC_STATE_PY] + 
                        _kalmanData.R[1][2]*_kalmanData.S[KC_STATE_PZ]);

            state.z = _kalmanData.S[KC_STATE_Z];

            state.dz = _kalmanData.R[2][0]*_kalmanData.S[KC_STATE_PX] + 
                _kalmanData.R[2][1]*_kalmanData.S[KC_STATE_PY] + 
                _kalmanData.R[2][2]*_kalmanData.S[KC_STATE_PZ];

            state.phi = RADIANS_TO_DEGREES *
                atan2f(2*(_kalmanData.q[2]*_kalmanData.q[3]+_kalmanData.q[0]*
                            _kalmanData.q[1]) ,
                        _kalmanData.q[0]*_kalmanData.q[0] -
                        _kalmanData.q[1]*_kalmanData.q[1] -
                        _kalmanData.q[2]*_kalmanData.q[2] +
                        _kalmanData.q[3]*_kalmanData.q[3]);

            state.theta = RADIANS_TO_DEGREES * 
                asinf(-2*(_kalmanData.q[1]*_kalmanData.q[3] -
                        _kalmanData.q[0]*_kalmanData.q[2]));

            state.psi = -RADIANS_TO_DEGREES *   // negate for nose-right positive
                atan2f(2*(_kalmanData.q[1]*_kalmanData.q[2]+_kalmanData.q[0]*
                            _kalmanData.q[3])
                        , _kalmanData.q[0]*_kalmanData.q[0] +
                        _kalmanData.q[1]*_kalmanData.q[1] -
                        _kalmanData.q[2]*_kalmanData.q[2] -
                        _kalmanData.q[3]*_kalmanData.q[3]);
        }

    private:

        static constexpr float GRAVITY_MAGNITUDE = 9.81;

        static constexpr float DEGREES_TO_RADIANS = PI / 180.0f;
        static constexpr float RADIANS_TO_DEGREES = 180.0f / PI;

        //We do get the measurements in 10x the motion pixels (experimentally measured)
        static constexpr float FLOW_RESOLUTION = 0.1;

        // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
        static constexpr float MAX_COVARIANCE = 100;
        static constexpr float MIN_COVARIANCE = 1e-6;

        // The bounds on states, these shouldn't be hit...
        static constexpr float MAX_POSITITON = 100; //meters
        static constexpr float MAX_VELOCITY = 10; //meters per second

        // Small number epsilon, to prevent dividing by zero
        static constexpr float EPSILON = 1e-6f;

        // the reversion of pitch and roll to zero
        static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

        static const int MAX_ITER = 2;

        static constexpr float UPPER_BOUND = 100;
        static constexpr float LOWER_BOUND = -100;

        typedef struct {
            Axis3f sum;
            uint32_t count;
            float conversionFactor;

            Axis3f subSample;
        } Axis3fSubSampler_t;


        // The parameters used by the filter
        typedef struct {

            // Initial variances, uncertain of position, but know we're stationary and
            // roughly flat
            float stdDevInitialPosition_xy;
            float stdDevInitialPosition_z;
            float stdDevInitialVelocity;
            float stdDevInitialAttitude_rollpitch;
            float stdDevInitialAttitude_yaw;

            float procNoiseAcc_xy;
            float procNoiseAcc_z;
            float procNoiseVel;
            float procNoisePos;
            float procNoiseAtt;
            float measNoiseBaro;           // meters
            float measNoiseGyro_rollpitch; // radians per second
            float measNoiseGyro_yaw;       // radians per second

            float initialX;
            float initialY;
            float initialZ;

            // Initial yaw of the Crazyflie in radians.
            // 0 --- facing positive X
            // PI / 2 --- facing positive Y
            // PI --- facing negative X
            // 3 * PI / 2 --- facing negative Y
            float initialYaw;

        } params_t;

        params_t _params;

        kalmanCoreData_t _kalmanData;

        Axis3f _accLatest;
        Axis3f _gyroLatest;

        Axis3fSubSampler_t _accSubSampler;
        Axis3fSubSampler_t _gyroSubSampler;

        OutlierFilterTdoa _outlierFilterTdoa;

        float _predictedNX;
        float _predictedNY;

        float _measuredNX;
        float _measuredNY;

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

        static void GM_UWB(float e, float * GM_e)
        {
            float sigma = 2.0;
            float GM_dn = sigma + e*e;
            *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
        }

        static void GM_state(float e, float * GM_e)
        {
            float sigma = 1.5;
            float GM_dn = sigma + e*e;
            *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
        }

        void addProcessNoiseDt(float dt)
        {
            _kalmanData.P[KC_STATE_X][KC_STATE_X] += 
                powf(_params.procNoiseAcc_xy*dt*dt + _params.procNoiseVel*dt + 
                        _params.procNoisePos, 2);  // add process noise on position

            _kalmanData.P[KC_STATE_Y][KC_STATE_Y] += 
                powf(_params.procNoiseAcc_xy*dt*dt + _params.procNoiseVel*dt + 
                        _params.procNoisePos, 2);  // add process noise on position

            _kalmanData.P[KC_STATE_Z][KC_STATE_Z] += 
                powf(_params.procNoiseAcc_z*dt*dt + _params.procNoiseVel*dt + 
                        _params.procNoisePos, 2);  // add process noise on position

            _kalmanData.P[KC_STATE_PX][KC_STATE_PX] += 
                powf(_params.procNoiseAcc_xy*dt + 
                        _params.procNoiseVel, 2); // add process noise on velocity

            _kalmanData.P[KC_STATE_PY][KC_STATE_PY] += 
                powf(_params.procNoiseAcc_xy*dt + 
                        _params.procNoiseVel, 2); // add process noise on velocity

            _kalmanData.P[KC_STATE_PZ][KC_STATE_PZ] += 
                powf(_params.procNoiseAcc_z*dt + 
                        _params.procNoiseVel, 2); // add process noise on velocity

            _kalmanData.P[KC_STATE_D0][KC_STATE_D0] += 
                powf(_params.measNoiseGyro_rollpitch * dt + _params.procNoiseAtt, 2);
            _kalmanData.P[KC_STATE_D1][KC_STATE_D1] += 
                powf(_params.measNoiseGyro_rollpitch * dt + _params.procNoiseAtt, 2);
            _kalmanData.P[KC_STATE_D2][KC_STATE_D2] += 
                powf(_params.measNoiseGyro_yaw * dt + _params.procNoiseAtt, 2);

            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float p = 0.5f*_kalmanData.P[i][j] + 0.5f*_kalmanData.P[j][i];
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MIN_COVARIANCE;
                    } else {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = p;
                    }
                }
            }
        }

        void predictDt(
                float A[KC_STATE_DIM][KC_STATE_DIM],
                matrix_t * Am,
                matrix_t * tmpNN1m,
                matrix_t * tmpNN2m,
                Axis3f *acc, 
                Axis3f *gyro, 
                float dt, 
                bool quadIsFlying)
        {
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
             *       x, p, d are the quad's states
             * note that d (attitude error) is zero at the beginning of each iteration,
             * since error information is incorporated into R after each Kalman update.
             */

            // ====== DYNAMICS LINEARIZATION ======
            // Initialize as the identity
            A[KC_STATE_X][KC_STATE_X] = 1;
            A[KC_STATE_Y][KC_STATE_Y] = 1;
            A[KC_STATE_Z][KC_STATE_Z] = 1;

            A[KC_STATE_PX][KC_STATE_PX] = 1;
            A[KC_STATE_PY][KC_STATE_PY] = 1;
            A[KC_STATE_PZ][KC_STATE_PZ] = 1;

            A[KC_STATE_D0][KC_STATE_D0] = 1;
            A[KC_STATE_D1][KC_STATE_D1] = 1;
            A[KC_STATE_D2][KC_STATE_D2] = 1;

            // position from body-frame velocity
            A[KC_STATE_X][KC_STATE_PX] = _kalmanData.R[0][0]*dt;
            A[KC_STATE_Y][KC_STATE_PX] = _kalmanData.R[1][0]*dt;
            A[KC_STATE_Z][KC_STATE_PX] = _kalmanData.R[2][0]*dt;

            A[KC_STATE_X][KC_STATE_PY] = _kalmanData.R[0][1]*dt;
            A[KC_STATE_Y][KC_STATE_PY] = _kalmanData.R[1][1]*dt;
            A[KC_STATE_Z][KC_STATE_PY] = _kalmanData.R[2][1]*dt;

            A[KC_STATE_X][KC_STATE_PZ] = _kalmanData.R[0][2]*dt;
            A[KC_STATE_Y][KC_STATE_PZ] = _kalmanData.R[1][2]*dt;
            A[KC_STATE_Z][KC_STATE_PZ] = _kalmanData.R[2][2]*dt;

            // position from attitude error
            A[KC_STATE_X][KC_STATE_D0] = (_kalmanData.S[KC_STATE_PY]*_kalmanData.R[0][2] - 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[0][1])*dt;
            A[KC_STATE_Y][KC_STATE_D0] = (_kalmanData.S[KC_STATE_PY]*_kalmanData.R[1][2] - 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[1][1])*dt;
            A[KC_STATE_Z][KC_STATE_D0] = (_kalmanData.S[KC_STATE_PY]*_kalmanData.R[2][2] - 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[2][1])*dt;

            A[KC_STATE_X][KC_STATE_D1] = (- _kalmanData.S[KC_STATE_PX]*_kalmanData.R[0][2] + 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[0][0])*dt;
            A[KC_STATE_Y][KC_STATE_D1] = (- _kalmanData.S[KC_STATE_PX]*_kalmanData.R[1][2] + 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[1][0])*dt;
            A[KC_STATE_Z][KC_STATE_D1] = (- _kalmanData.S[KC_STATE_PX]*_kalmanData.R[2][2] + 
                    _kalmanData.S[KC_STATE_PZ]*_kalmanData.R[2][0])*dt;

            A[KC_STATE_X][KC_STATE_D2] = (_kalmanData.S[KC_STATE_PX]*_kalmanData.R[0][1] - 
                    _kalmanData.S[KC_STATE_PY]*_kalmanData.R[0][0])*dt;
            A[KC_STATE_Y][KC_STATE_D2] = (_kalmanData.S[KC_STATE_PX]*_kalmanData.R[1][1] - 
                    _kalmanData.S[KC_STATE_PY]*_kalmanData.R[1][0])*dt;
            A[KC_STATE_Z][KC_STATE_D2] = (_kalmanData.S[KC_STATE_PX]*_kalmanData.R[2][1] - 
                    _kalmanData.S[KC_STATE_PY]*_kalmanData.R[2][0])*dt;

            // body-frame velocity from body-frame velocity
            A[KC_STATE_PX][KC_STATE_PX] = 1; //drag negligible
            A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
            A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;

            A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
            A[KC_STATE_PY][KC_STATE_PY] = 1; //drag negligible
            A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;

            A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
            A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
            A[KC_STATE_PZ][KC_STATE_PZ] = 1; //drag negligible

            // body-frame velocity from attitude error
            A[KC_STATE_PX][KC_STATE_D0] =  0;
            A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*_kalmanData.R[2][2]*dt;
            A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*_kalmanData.R[2][1]*dt;

            A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*_kalmanData.R[2][2]*dt;
            A[KC_STATE_PY][KC_STATE_D1] =  0;
            A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*_kalmanData.R[2][0]*dt;

            A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*_kalmanData.R[2][1]*dt;
            A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*_kalmanData.R[2][0]*dt;
            A[KC_STATE_PZ][KC_STATE_D2] =  0;


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
            float d0 = gyro->x*dt/2;
            float d1 = gyro->y*dt/2;
            float d2 = gyro->z*dt/2;

            A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
            A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
            A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

            A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
            A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
            A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

            A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
            A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
            A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

            // ====== COVARIANCE UPDATE ======
            device_mat_mult(Am, &_kalmanData.Pm, tmpNN1m); // A P
            device_mat_trans(Am, tmpNN2m); // A'
            device_mat_mult(tmpNN1m, tmpNN2m, &_kalmanData.Pm); // A P A'
            // Process noise is added after the return from the prediction step

            // ====== PREDICTION STEP ======
            // The prediction depends on whether we're on the ground, or in flight.
            // When flying, the accelerometer directly measures thrust (hence is useless
            // to estimate body angle while flying)

            float dx, dy, dz;
            float tmpSPX, tmpSPY, tmpSPZ;
            float zacc;

            float dt2 = dt*dt;

            if (quadIsFlying) { // only acceleration in z direction

                // Use accelerometer and not commanded thrust, as this has
                // proper physical units
                zacc = acc->z;

                // position updates in the body frame (will be rotated to inertial frame)
                dx = _kalmanData.S[KC_STATE_PX] * dt;
                dy = _kalmanData.S[KC_STATE_PY] * dt;
                dz = _kalmanData.S[KC_STATE_PZ] * dt + zacc * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _kalmanData.S[KC_STATE_X] += _kalmanData.R[0][0] * dx + 
                    _kalmanData.R[0][1] * dy + _kalmanData.R[0][2] * dz;
                _kalmanData.S[KC_STATE_Y] += _kalmanData.R[1][0] * dx + 
                    _kalmanData.R[1][1] * dy + _kalmanData.R[1][2] * dz;
                _kalmanData.S[KC_STATE_Z] += _kalmanData.R[2][0] * dx + 
                    _kalmanData.R[2][1] * dy + _kalmanData.R[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                tmpSPX = _kalmanData.S[KC_STATE_PX];
                tmpSPY = _kalmanData.S[KC_STATE_PY];
                tmpSPZ = _kalmanData.S[KC_STATE_PZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _kalmanData.S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y *
                        tmpSPZ - GRAVITY_MAGNITUDE * _kalmanData.R[2][0]);
                _kalmanData.S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - 
                        GRAVITY_MAGNITUDE * _kalmanData.R[2][1]);
                _kalmanData.S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _kalmanData.R[2][2]);
            }
            else {
                // Acceleration can be in any direction, as measured by the
                // accelerometer. This occurs, eg. in freefall or while being carried.

                // position updates in the body frame (will be rotated to inertial frame)
                dx = _kalmanData.S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
                dy = _kalmanData.S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
                dz = _kalmanData.S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _kalmanData.S[KC_STATE_X] += _kalmanData.R[0][0] * dx 
                    + _kalmanData.R[0][1] * dy + _kalmanData.R[0][2] * dz;
                _kalmanData.S[KC_STATE_Y] += _kalmanData.R[1][0] * dx + 
                    _kalmanData.R[1][1] * dy + _kalmanData.R[1][2] * dz;
                _kalmanData.S[KC_STATE_Z] += _kalmanData.R[2][0] * dx + 
                    _kalmanData.R[2][1] * dy + _kalmanData.R[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                tmpSPX = _kalmanData.S[KC_STATE_PX];
                tmpSPY = _kalmanData.S[KC_STATE_PY];
                tmpSPZ = _kalmanData.S[KC_STATE_PZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _kalmanData.S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY -
                        gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * _kalmanData.R[2][0]);
                _kalmanData.S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * 
                        tmpSPZ - GRAVITY_MAGNITUDE * _kalmanData.R[2][1]);
                _kalmanData.S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _kalmanData.R[2][2]);
            }

            // attitude update (rotate by gyroscope), we do this in quaternions
            // this is the gyroscope angular velocity integrated over the sample period
            float dtwx = dt*gyro->x;
            float dtwy = dt*gyro->y;
            float dtwz = dt*gyro->z;

            // compute the quaternion values in [w,x,y,z] order
            float angle = device_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPSILON;
            float ca = device_cos(angle/2.0f);
            float sa = device_sin(angle/2.0f);
            float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

            float tmpq0;
            float tmpq1;
            float tmpq2;
            float tmpq3;

            // rotate the quad's attitude by the delta quaternion vector computed above

            tmpq0 = dq[0]*_kalmanData.q[0] - dq[1]*_kalmanData.q[1] - 
                dq[2]*_kalmanData.q[2] - dq[3]*_kalmanData.q[3];

            tmpq1 = dq[1]*_kalmanData.q[0] + dq[0]*_kalmanData.q[1] + 
                dq[3]*_kalmanData.q[2] - dq[2]*_kalmanData.q[3];

            tmpq2 = dq[2]*_kalmanData.q[0] - dq[3]*_kalmanData.q[1] + 
                dq[0]*_kalmanData.q[2] + dq[1]*_kalmanData.q[3];

            tmpq3 = dq[3]*_kalmanData.q[0] + dq[2]*_kalmanData.q[1] - 
                dq[1]*_kalmanData.q[2] + dq[0]*_kalmanData.q[3];

            if (! quadIsFlying) {

                float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                tmpq0 = keep * tmpq0 + 
                    ROLLPITCH_ZERO_REVERSION * _kalmanData.initialQuaternion[0];
                tmpq1 = keep * tmpq1 + 
                    ROLLPITCH_ZERO_REVERSION * _kalmanData.initialQuaternion[1];
                tmpq2 = keep * tmpq2 + 
                    ROLLPITCH_ZERO_REVERSION * _kalmanData.initialQuaternion[2];
                tmpq3 = keep * tmpq3 + 
                    ROLLPITCH_ZERO_REVERSION * _kalmanData.initialQuaternion[3];
            }

            // normalize and store the result
            float norm = device_sqrt(
                    tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPSILON;

            _kalmanData.q[0] = tmpq0/norm; 
            _kalmanData.q[1] = tmpq1/norm; 
            _kalmanData.q[2] = tmpq2/norm; 
            _kalmanData.q[3] = tmpq3/norm;


            _kalmanData.isUpdated = true;
        }

        void scalarUpdate(
                matrix_t *Hm, 
                float error, 
                float stdMeasNoise)
        {
            // The Kalman gain as a column vector
            static float K[KC_STATE_DIM];
            static matrix_t Km = {KC_STATE_DIM, 1, (float *)K};

            // Temporary matrices for the covariance updates
            static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
            static matrix_t tmpNN1m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
            static matrix_t tmpNN2m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN2d
            };

            static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
            static matrix_t tmpNN3m = {
                KC_STATE_DIM, KC_STATE_DIM, tmpNN3d
            };

            static float HTd[KC_STATE_DIM * 1];
            static matrix_t HTm = {KC_STATE_DIM, 1, HTd};

            static float PHTd[KC_STATE_DIM * 1];
            static matrix_t PHTm = {KC_STATE_DIM, 1, PHTd};

            scalarUpdate(Hm, &HTm, &Km, PHTd,
                    K, tmpNN1d, &PHTm, &tmpNN1m, &tmpNN2m, &tmpNN3m, 
                    error, stdMeasNoise);
        }

        void scalarUpdate(
                matrix_t *Hm, 
                matrix_t *HTm, 
                matrix_t *Km, 
                float * PHTd, 
                float * K, 
                float * tmpNN1d, 
                matrix_t *PHTm, 
                matrix_t *tmpNN1m, 
                matrix_t *tmpNN2m, 
                matrix_t *tmpNN3m, 
                float error, 
                float stdMeasNoise)
        {
            // ====== INNOVATION COVARIANCE ======

            device_mat_trans(Hm, HTm);
            device_mat_mult(&_kalmanData.Pm, HTm, PHTm); // PH'
            float R = stdMeasNoise*stdMeasNoise;
            float HPHR = R; // HPH' + R
            for (int i=0; i<KC_STATE_DIM; i++) { 

                // Add the element of HPH' to the above

                // this obviously only works if the update is scalar (as in this function)
                HPHR += Hm->pData[i]*PHTd[i]; 
            }

            // ====== MEASUREMENT UPDATE ======
            // Calculate the Kalman gain and perform the state update
            for (int i=0; i<KC_STATE_DIM; i++) {
                K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                _kalmanData.S[i] = _kalmanData.S[i] + K[i] * error; // state update
            }


            // ====== COVARIANCE UPDATE ======
            device_mat_mult(Km, Hm, tmpNN1m); // KH
            for (int i=0; i<KC_STATE_DIM; i++) { 
                tmpNN1d[KC_STATE_DIM*i+i] -= 1; 
            } // KH - I
            device_mat_trans(tmpNN1m, tmpNN2m); // (KH - I)'
            device_mat_mult(tmpNN1m, &_kalmanData.Pm, tmpNN3m); // (KH - I)*P
            device_mat_mult(tmpNN3m, tmpNN2m, &_kalmanData.Pm); // (KH - I)*P*(KH - I)'

            // add the measurement variance and ensure boundedness and symmetry
            // TODO: Why would it hit these bounds? Needs to be investigated.
            for (int i=0; i<KC_STATE_DIM; i++) {
                for (int j=i; j<KC_STATE_DIM; j++) {
                    float v = K[i] * R * K[j];

                    // add measurement noise
                    float p = 0.5f*_kalmanData.P[i][j] + 0.5f*_kalmanData.P[j][i] + v; 
                    if (isnan(p) || p > MAX_COVARIANCE) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MAX_COVARIANCE;
                    } else if ( i==j && p < MIN_COVARIANCE ) {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = MIN_COVARIANCE;
                    } else {
                        _kalmanData.P[i][j] = _kalmanData.P[j][i] = p;
                    }
                }
            }

            _kalmanData.isUpdated = true;
        }

        // Measurement model where the measurement is the absolute height
        void updateWithAbsoluteHeight(heightMeasurement_t* height) 
        {
            float h[KC_STATE_DIM] = {};
            matrix_t H = {1, KC_STATE_DIM, h};
            h[KC_STATE_Z] = 1;
            scalarUpdate(
                    &H, height->height - _kalmanData.S[KC_STATE_Z], height->stdDev);
        }

        // Measurement model where the measurement is the distance to a known
        // point in space
        void updateWithDistance(distanceMeasurement_t* d) 
        {
            // a measurement of distance to point (x, y, z)
            float h[KC_STATE_DIM] = {};
            matrix_t H = {1, KC_STATE_DIM, h};

            float dx = _kalmanData.S[KC_STATE_X] - d->x;
            float dy = _kalmanData.S[KC_STATE_Y] - d->y;
            float dz = _kalmanData.S[KC_STATE_Z] - d->z;

            float measuredDistance = d->distance;

            float predictedDistance = device_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
            if (predictedDistance != 0.0f) {

                // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The
                // derivative dz/dX gives h.
                h[KC_STATE_X] = dx/predictedDistance;
                h[KC_STATE_Y] = dy/predictedDistance;
                h[KC_STATE_Z] = dz/predictedDistance;
            } else {
                // Avoid divide by zero
                h[KC_STATE_X] = 1.0f;
                h[KC_STATE_Y] = 0.0f;
                h[KC_STATE_Z] = 0.0f;
            }

            scalarUpdate(&H, measuredDistance-predictedDistance, d->stdDev);
        }

        void updateWithFlow(const flowMeasurement_t *flow) 
        {
            const Axis3f *gyro = &_gyroLatest;

            // Inclusion of flow measurements in the EKF done by two scalar updates

            // ~~~ Camera constants ~~~
            // The angle of aperture is guessed from the raw data register and
            // thankfully look to be symmetric

            float Npix = 35.0;                      // [pixels] (same in x and y)
            //float thetapix = DEGREES_TO_RADIANS * 4.0f;     // [rad]    (same in x and y)

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
            //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * 
            //  S[KC_STATE_PZ];
            //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * 
            //  S[KC_STATE_PZ];


            float dx_g = _kalmanData.S[KC_STATE_PX];
            float dy_g = _kalmanData.S[KC_STATE_PY];
            float z_g = 0.0;
            // Saturate elevation in prediction and correction to avoid singularities
            if ( _kalmanData.S[KC_STATE_Z] < 0.1f ) {
                z_g = 0.1;
            } else {
                z_g = _kalmanData.S[KC_STATE_Z];
            }

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            float hx[KC_STATE_DIM] = {};
            matrix_t Hx = {1, KC_STATE_DIM, hx};
            _predictedNX = (flow->dt * Npix / thetapix ) * 
                ((dx_g * _kalmanData.R[2][2] / z_g) - omegay_b);
            _measuredNX = flow->dpixelx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_kalmanData.R[2][2] * dx_g) / (-z_g * z_g));
            hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * 
                (_kalmanData.R[2][2] / z_g);

            //First update
            scalarUpdate(&Hx, (_measuredNX-_predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION);

            // ~~~ Y velocity prediction and update ~~~
            float hy[KC_STATE_DIM] = {};
            matrix_t Hy = {1, KC_STATE_DIM, hy};
            _predictedNY = (flow->dt * Npix / thetapix ) * 
                ((dy_g * _kalmanData.R[2][2] / z_g) + omegax_b);
            _measuredNY = flow->dpixely*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_kalmanData.R[2][2] * dy_g) / (-z_g * z_g));
            hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (_kalmanData.R[2][2] / z_g);

            // Second update
            scalarUpdate(
                    &Hy, (_measuredNY-_predictedNY), flow->stdDevY*FLOW_RESOLUTION);
        }

        void updateWithPose(poseMeasurement_t *pose)
        {
            // a direct measurement of states x, y, and z, and orientation do a
            // scalar update for each state, since this should be faster than
            // updating all together
            for (int i=0; i<3; i++) {
                float h[KC_STATE_DIM] = {};
                matrix_t H = {1, KC_STATE_DIM, h};
                h[KC_STATE_X+i] = 1;
                scalarUpdate(
                        &H, pose->pos[i] - _kalmanData.S[KC_STATE_X+i], pose->stdDevPos);
            }

            // compute orientation error
            const quat_t q_ekf = 
                mkquat(
                        _kalmanData.q[1], 
                        _kalmanData.q[2], 
                        _kalmanData.q[3], 
                        _kalmanData.q[0]);
            const quat_t q_measured = 
                mkquat(pose->quat.x, pose->quat.y, pose->quat.z, pose->quat.w);
            const quat_t q_residual = qqmul(qinv(q_ekf), q_measured);

            // small angle approximation, see eq. 141 in
            // http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
            struct vec const err_quat = 
                vscl(2.0f / q_residual.w, quatimagpart(q_residual));

            // do a scalar update for each state
            {
                float h[KC_STATE_DIM] = {};
                matrix_t H = {1, KC_STATE_DIM, h};
                h[KC_STATE_D0] = 1;
                scalarUpdate(&H, err_quat.x, pose->stdDevQuat);
                h[KC_STATE_D0] = 0;

                h[KC_STATE_D1] = 1;
                scalarUpdate(&H, err_quat.y, pose->stdDevQuat);
                h[KC_STATE_D1] = 0;

                h[KC_STATE_D2] = 1;
                scalarUpdate(&H, err_quat.z, pose->stdDevQuat);
            }
        }

        void updateWithPosition(positionMeasurement_t *xyz)
        {
            // a direct measurement of states x, y, and z do a scalar update
            // for each state, since this should be faster than updating all
            // together
            for (int i=0; i<3; i++) {
                float h[KC_STATE_DIM] = {};
                matrix_t H = {1, KC_STATE_DIM, h};
                h[KC_STATE_X+i] = 1;
                scalarUpdate(&H, xyz->pos[i] -
                        _kalmanData.S[KC_STATE_X+i], xyz->stdDev); 
            }
        }

        void updateWithTdoa( tdoaMeasurement_t *tdoa, const uint32_t nowMs)
        {
            /**
             * Measurement equation:
             * dR = dT + d1 - d0
             */

            float measurement = tdoa->distanceDiff;

            // predict based on current state
            float x = _kalmanData.S[KC_STATE_X];
            float y = _kalmanData.S[KC_STATE_Y];
            float z = _kalmanData.S[KC_STATE_Z];

            float x1 = tdoa->anchorPositions[1].x;
            float y1 = tdoa->anchorPositions[1].y; 
            float z1 = tdoa->anchorPositions[1].z;
            float x0 = tdoa->anchorPositions[0].x;
            float y0 = tdoa->anchorPositions[0].y;
            float z0 = tdoa->anchorPositions[0].z;

            float dx1 = x - x1;
            float dy1 = y - y1;
            float dz1 = z - z1;

            float dy0 = y - y0;
            float dx0 = x - x0;
            float dz0 = z - z0;

            float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
            float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

            float predicted = d1 - d0;
            float error = measurement - predicted;

            float h[KC_STATE_DIM] = {};
            matrix_t H = {1, KC_STATE_DIM, h};

            if ((d0 != 0.0f) && (d1 != 0.0f)) {
                h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
                h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
                h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

                bool sampleIsGood = 
                    _outlierFilterTdoa.validateIntegrator(tdoa, error, nowMs);

                if (sampleIsGood) {
                    scalarUpdate(&H, error, tdoa->stdDev);
                }
            }
        }

        void updateWithTof(tofMeasurement_t *tof)
        {
            // Updates the filter with a measured distance in the zb direction using the
            float h[KC_STATE_DIM] = {};
            matrix_t H = {1, KC_STATE_DIM, h};

            // Only update the filter if the measurement is reliable 
            // (\hat{h} -> infty when R[2][2] -> 0)
            if (fabs(_kalmanData.R[2][2]) > 0.1f && _kalmanData.R[2][2] > 0) {
                float angle = 
                    fabsf(acosf(_kalmanData.R[2][2])) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f);
                if (angle < 0.0f) {
                    angle = 0.0f;
                }
                float predictedDistance = _kalmanData.S[KC_STATE_Z] / cosf(angle);
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
                h[KC_STATE_Z] = 1 / cosf(angle); 

                // Scalar update
                scalarUpdate(
                        &H, measuredDistance-predictedDistance, tof->stdDev);
            }
        }

        void updateWithYawError(yawErrorMeasurement_t *error)
        {
            float h[KC_STATE_DIM] = {};
            matrix_t H = {1, KC_STATE_DIM, h};

            h[KC_STATE_D2] = 1;
            scalarUpdate(&H, _kalmanData.S[KC_STATE_D2] - error->yawError, error->stdDev); 
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

        static void device_mat_trans(const matrix_t * pSrc, matrix_t * pDst); 

        static void device_mat_mult(
                const matrix_t * pSrcA, const matrix_t * pSrcB,
                matrix_t * pDst);

        static float device_cos(const float x);

        static float device_sin(const float x);

        static float device_sqrt(const float32_t in);
};
