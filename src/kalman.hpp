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

            _ekf.x[STATE_X] = 0;
            _ekf.x[STATE_Y] = 0;
            _ekf.x[STATE_Z] = 0;

            // Reset the attitude quaternion
            _initialQuaternion[0] = 1;
            _initialQuaternion[1] = 0;
            _initialQuaternion[2] = 0;
            _initialQuaternion[3] = 0;

            for (int i = 0; i < 4; i++) { 
                _quat[i] = _initialQuaternion[i]; 
            }

            for(int i=0; i<3; i++) { 
                for(int j=0; j<3; j++) { 
                    _rotmat[i][j] = i==j ? 1 : 0; 
                }
            }

            _ekf.init(MIN_COVARIANCE, MAX_COVARIANCE);

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

            _ekf.addCovarianceNoise(pinit);

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
            A[STATE_X][STATE_D0] = (_ekf.x[STATE_VY]*_rotmat[0][2] - 
                    _ekf.x[STATE_VZ]*_rotmat[0][1])*dt;
            A[STATE_Y][STATE_D0] = (_ekf.x[STATE_VY]*_rotmat[1][2] - 
                    _ekf.x[STATE_VZ]*_rotmat[1][1])*dt;
            A[STATE_Z][STATE_D0] = (_ekf.x[STATE_VY]*_rotmat[2][2] - 
                    _ekf.x[STATE_VZ]*_rotmat[2][1])*dt;

            A[STATE_X][STATE_D1] = (- _ekf.x[STATE_VX]*_rotmat[0][2] + 
                    _ekf.x[STATE_VZ]*_rotmat[0][0])*dt;
            A[STATE_Y][STATE_D1] = (- _ekf.x[STATE_VX]*_rotmat[1][2] + 
                    _ekf.x[STATE_VZ]*_rotmat[1][0])*dt;
            A[STATE_Z][STATE_D1] = (- _ekf.x[STATE_VX]*_rotmat[2][2] + 
                    _ekf.x[STATE_VZ]*_rotmat[2][0])*dt;

            A[STATE_X][STATE_D2] = (_ekf.x[STATE_VX]*_rotmat[0][1] - 
                    _ekf.x[STATE_VY]*_rotmat[0][0])*dt;
            A[STATE_Y][STATE_D2] = (_ekf.x[STATE_VX]*_rotmat[1][1] - 
                    _ekf.x[STATE_VY]*_rotmat[1][0])*dt;
            A[STATE_Z][STATE_D2] = (_ekf.x[STATE_VX]*_rotmat[2][1] - 
                    _ekf.x[STATE_VY]*_rotmat[2][0])*dt;

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

            _ekf.updateCovariance(A);

            const float dt2 = dt * dt;

            if (isFlying) { // only acceleration in z direction

                // Use accelerometer and not commanded thrust, as this has
                // proper physical units
                const float zacc = accel->z;

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _ekf.x[STATE_VX] * dt;
                const float dy = _ekf.x[STATE_VY] * dt;
                const float dz = _ekf.x[STATE_VZ] * dt + zacc * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _ekf.x[STATE_X] += _rotmat[0][0] * dx + 
                    _rotmat[0][1] * dy + _rotmat[0][2] * dz;
                _ekf.x[STATE_Y] += _rotmat[1][0] * dx + 
                    _rotmat[1][1] * dy + _rotmat[1][2] * dz;
                _ekf.x[STATE_Z] += _rotmat[2][0] * dx + 
                    _rotmat[2][1] * dy + _rotmat[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                const float tmpSPX = _ekf.x[STATE_VX];
                const float tmpSPY = _ekf.x[STATE_VY];
                const float tmpSPZ = _ekf.x[STATE_VZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _ekf.x[STATE_VX] += dt * (gyro->z * tmpSPY - gyro->y *
                        tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][0]);
                _ekf.x[STATE_VY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - 
                        GRAVITY_MAGNITUDE * _rotmat[2][1]);
                _ekf.x[STATE_VZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * 
                        tmpSPY - GRAVITY_MAGNITUDE * _rotmat[2][2]);
            }
            else {
                // Acceleration can be in any direction, as measured by the
                // accelerometer. This occurs, eg. in freefall or while being carried.

                // position updates in the body frame (will be rotated to inertial frame)
                const float dx = _ekf.x[STATE_VX] * dt + accel->x * dt2 / 2.0f;
                const float dy = _ekf.x[STATE_VY] * dt + accel->y * dt2 / 2.0f;
                const float dz = _ekf.x[STATE_VZ] * dt + accel->z * dt2 / 2.0f; 
                // thrust can only be produced in the body's Z direction

                // position update
                _ekf.x[STATE_X] += _rotmat[0][0] * dx 
                    + _rotmat[0][1] * dy + _rotmat[0][2] * dz;
                _ekf.x[STATE_Y] += _rotmat[1][0] * dx + 
                    _rotmat[1][1] * dy + _rotmat[1][2] * dz;
                _ekf.x[STATE_Z] += _rotmat[2][0] * dx + 
                    _rotmat[2][1] * dy + _rotmat[2][2] * dz - 
                    GRAVITY_MAGNITUDE * dt2 / 2.0f;

                // keep previous time step's state for the update
                const float tmpSPX = _ekf.x[STATE_VX];
                const float tmpSPY = _ekf.x[STATE_VY];
                const float tmpSPZ = _ekf.x[STATE_VZ];

                // body-velocity update: accelerometers - gyros cross velocity
                // - gravity in body frame
                _ekf.x[STATE_VX] += dt * (accel->x + gyro->z * tmpSPY -
                        gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][0]);
                _ekf.x[STATE_VY] += dt * (accel->y - gyro->z * tmpSPX + gyro->x * 
                        tmpSPZ - GRAVITY_MAGNITUDE * _rotmat[2][1]);
                _ekf.x[STATE_VZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * 
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

                _ekf.addCovarianceNoise(noise);
                _ekf.enforceSymmetry();

                _lastProcessNoiseUpdateMs = nowMs;
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
            const float v0 = _ekf.x[STATE_D0];
            const float v1 = _ekf.x[STATE_D1];
            const float v2 = _ekf.x[STATE_D2];

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
            _ekf.x[STATE_D0] = 0;
            _ekf.x[STATE_D1] = 0;
            _ekf.x[STATE_D2] = 0;

            _ekf.enforceSymmetry();

            _isUpdated = false;
        }


        bool isStateWithinBounds(void) 
        {
            for (int i = 0; i < 3; i++) {

                if (MAX_VELOCITY > 0.0f) {
                    if (_ekf.x[STATE_VX + i] > MAX_VELOCITY) {
                        return false;
                    } else if (_ekf.x[STATE_VX + i] < -MAX_VELOCITY) {
                        return false;
                    }
                }
            }

            return true;
        }

        void getVehicleState(vehicleState_t & state)
        {
            state.x = _ekf.x[STATE_X];

            state.dx = _rotmat[0][0]*_ekf.x[STATE_VX] + 
                _rotmat[0][1]*_ekf.x[STATE_VY] + 
                _rotmat[0][2]*_ekf.x[STATE_VZ];

            state.y = _ekf.x[STATE_Y];

            // Negate for rightward positive
            state.dy = -(_rotmat[1][0]*_ekf.x[STATE_VX] + 
                    _rotmat[1][1]*_ekf.x[STATE_VY] + 
                    _rotmat[1][2]*_ekf.x[STATE_VZ]);

            state.z = _ekf.x[STATE_Z];

            state.dz = _rotmat[2][0]*_ekf.x[STATE_VX] + 
                _rotmat[2][1]*_ekf.x[STATE_VY] + 
                _rotmat[2][2]*_ekf.x[STATE_VZ];

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


            float dx_g = _ekf.x[STATE_VX];
            float dy_g = _ekf.x[STATE_VY];
            float z_g = 0.0;
            // Saturate elevation in prediction and correction to avoid singularities
            if ( _ekf.x[STATE_Z] < 0.1f ) {
                z_g = 0.1;
            } else {
                z_g = _ekf.x[STATE_Z];
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
            _ekf.updateWithScalar(hx, (_measuredNX-_predictedNX), 
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
            _ekf.updateWithScalar(hy, (_measuredNY-_predictedNY),
                    flow->stdDevY*FLOW_RESOLUTION);

            _isUpdated = true;
        }

        void updateWithTof(tofMeasurement_t *tof)
        {
            // Updates the filter with a measured distance in the zb direction using the
            float h[STATE_DIM] = {};

            // Only update the filter if the measurement is reliable 
            // (\hat{h} -> infty when R[2][2] -> 0)
            if (fabs(_rotmat[2][2]) > 0.1f && _rotmat[2][2] > 0) {
                float angle = 
                    fabsf(acosf(_rotmat[2][2])) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f);
                if (angle < 0.0f) {
                    angle = 0.0f;
                }
                float predictedDistance = _ekf.x[STATE_Z] / cosf(angle);
                float measuredDistance = tof->distance; // [m]

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                h[STATE_Z] = 1 / cosf(angle); 

                _ekf.updateWithScalar(h, measuredDistance-predictedDistance, tof->stdDev);

                _isUpdated = true;
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

        EKF _ekf;

        // Tracks whether an update to the state has been made, and the state
        // therefore requires finalization
        bool _isUpdated;

        uint32_t _lastPredictionMs;
        uint32_t _lastProcessNoiseUpdateMs;

        static float device_cos(const float x);

        static float device_sin(const float x);

        static float device_sqrt(const float32_t in);
};
