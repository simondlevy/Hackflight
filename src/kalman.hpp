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
            axis3fSubSamplerInit(&_accSubSampler, GRAVITY);
            axis3fSubSamplerInit(&_gyroSubSampler, DEGREES_TO_RADIANS);

            _outlierFilterTdoa.reset();

            ekf_init();

            // Reset the attitude quaternion
            _q0 = _qinit0 = 1;
            _q1 = _qinit1 = 0;
            _q2 = _qinit2 = 0;
            _q3 = _qinit3 = 0;

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

            ekf_addCovarianceNoise(pinit);

            _isUpdated = false;
            _lastPredictionMs = nowMs;
            _lastProcessNoiseUpdateMs = nowMs;
        }

        void predict(const uint32_t nowMs, bool isFlying) 
        {
            axis3fSubSamplerFinalize(&_accSubSampler);
            axis3fSubSamplerFinalize(&_gyroSubSampler);

            const float dt = (nowMs - _lastPredictionMs) / 1000.0f;

            const Axis3f * accel = &_accSubSampler.subSample;
            const Axis3f * gyro = &_gyroSubSampler.subSample;

            const float d0 = gyro->x*dt/2;
            const float d1 = gyro->y*dt/2;
            const float d2 = gyro->z*dt/2;

            const float vx = _x[STATE_VX];
            const float vy = _x[STATE_VY];
            const float vz = _x[STATE_VZ];

            // The linearized Jacobean matrix
            static float F[STATE_DIM][STATE_DIM];

            // position
            F[STATE_X][STATE_X] = 1;
            F[STATE_Y][STATE_Y] = 1;
            F[STATE_Z][STATE_Z] = 1;

            // position from body-frame velocity
            F[STATE_X][STATE_VX] = _r00*dt;
            F[STATE_Y][STATE_VX] = _r10*dt;
            F[STATE_Z][STATE_VX] = _r20*dt;

            F[STATE_X][STATE_VY] = _r01*dt;
            F[STATE_Y][STATE_VY] = _r11*dt;
            F[STATE_Z][STATE_VY] = _r21*dt;

            F[STATE_X][STATE_VZ] = _r02*dt;
            F[STATE_Y][STATE_VZ] = _r12*dt;
            F[STATE_Z][STATE_VZ] = _r22*dt;

            // position from attitude error
            F[STATE_X][STATE_D0] = (vy*_r02 - vz*_r01)*dt;
            F[STATE_Y][STATE_D0] = (vy*_r12 - vz*_r11)*dt;
            F[STATE_Z][STATE_D0] = (vy*_r22 - vz*_r21)*dt;

            F[STATE_X][STATE_D1] = (-vx*_r02 + vz*_r00)*dt;
            F[STATE_Y][STATE_D1] = (-vx*_r12 + vz*_r10)*dt;
            F[STATE_Z][STATE_D1] = (-vx*_r22 + vz*_r20)*dt;

            F[STATE_X][STATE_D2] = (vx*_r01 - vy*_r00)*dt;
            F[STATE_Y][STATE_D2] = (vx*_r11 - vy*_r10)*dt;
            F[STATE_Z][STATE_D2] = (vx*_r21 - vy*_r20)*dt;

            // body-frame velocity from body-frame velocity
            F[STATE_VX][STATE_VX] = 1; //drag negligible
            F[STATE_VY][STATE_VX] =-gyro->z*dt;
            F[STATE_VZ][STATE_VX] = gyro->y*dt;

            F[STATE_VX][STATE_VY] = gyro->z*dt;
            F[STATE_VY][STATE_VY] = 1; //drag negligible
            F[STATE_VZ][STATE_VY] =-gyro->x*dt;

            F[STATE_VX][STATE_VZ] =-gyro->y*dt;
            F[STATE_VY][STATE_VZ] = gyro->x*dt;
            F[STATE_VZ][STATE_VZ] = 1; //drag negligible

            // body-frame velocity from attitude error
            F[STATE_VX][STATE_D0] =  0;
            F[STATE_VY][STATE_D0] = -GRAVITY*_r22*dt;
            F[STATE_VZ][STATE_D0] =  GRAVITY*_r21*dt;

            F[STATE_VX][STATE_D1] =  GRAVITY*_r22*dt;
            F[STATE_VY][STATE_D1] =  0;
            F[STATE_VZ][STATE_D1] = -GRAVITY*_r20*dt;

            F[STATE_VX][STATE_D2] = -GRAVITY*_r21*dt;
            F[STATE_VY][STATE_D2] =  GRAVITY*_r20*dt;
            F[STATE_VZ][STATE_D2] =  0;

            F[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
            F[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
            F[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

            F[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
            F[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
            F[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

            F[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
            F[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
            F[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

            ekf_predict(F);

            const float dt2 = dt * dt;

            // keep previous time step's state for the update
            const float tmpSPX = _x[STATE_VX];
            const float tmpSPY = _x[STATE_VY];
            const float tmpSPZ = _x[STATE_VZ];

            // position updates in the body frame (will be rotated to inertial frame)
            const float dx = _x[STATE_VX] * dt + (isFlying ? 0 : accel->x * dt2 / 2.0f);
            const float dy = _x[STATE_VY] * dt + (isFlying ? 0 : accel->y * dt2 / 2.0f);

            // thrust can only be produced in the body's Z direction
            const float dz = _x[STATE_VZ] * dt + accel->z * dt2 / 2.0f; 

            // position update
            _x[STATE_X] += _r00 * dx + _r01 * dy + _r02 * dz;
            _x[STATE_Y] += _r10 * dx + _r11 * dy + _r12 * dz;
            _x[STATE_Z] += _r20 * dx + _r21 * dy + _r22 * dz - 
                GRAVITY * dt2 / 2.0f;

            const float accelx = isFlying ? 0 : accel->x;
            const float accely = isFlying ? 0 : accel->y;

            // body-velocity update: accelerometers - gyros cross velocity
            // - gravity in body frame

            _x[STATE_VX] += dt * (accelx + gyro->z * tmpSPY - gyro->y * tmpSPZ
                    - GRAVITY * _r20);

            _x[STATE_VY] += dt * (accely - gyro->z * tmpSPX + gyro->x * tmpSPZ
                    - GRAVITY * _r21);

            _x[STATE_VZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY
                    - GRAVITY * _r22);

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

            float tmpq0 = dq[0]*_q0 - dq[1]*_q1 - dq[2]*_q2 - dq[3]*_q3;
            float tmpq1 = dq[1]*_q0 + dq[0]*_q1 + dq[3]*_q2 - dq[2]*_q3;
            float tmpq2 = dq[2]*_q0 - dq[3]*_q1 + dq[0]*_q2 + dq[1]*_q3;
            float tmpq3 = dq[3]*_q0 + dq[2]*_q1 - dq[1]*_q2 + dq[0]*_q3;

            if (!isFlying) {

                const float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

                tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * _qinit0;
                tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * _qinit1;
                tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * _qinit2;
                tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * _qinit3;
            }

            // normalize and store the result
            const float norm = device_sqrt(
                    tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPSILON;

            _q0 = tmpq0/norm; 
            _q1 = tmpq1/norm; 
            _q2 = tmpq2/norm; 
            _q3 = tmpq3/norm;

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

                ekf_addCovarianceNoise(noise);
                ekf_enforceSymmetry();

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
            // Only finalize if data is updated
            if (! _isUpdated) {
                return;
            }

            // Incorporate the attitude error (Kalman filter state) with the attitude
            const float v0 = _x[STATE_D0];
            const float v1 = _x[STATE_D1];
            const float v2 = _x[STATE_D2];

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
                const float tmpq0 = dq[0] * _q0 - dq[1] * _q1 - 
                    dq[2] * _q2 - dq[3] * _q3;
                const float tmpq1 = dq[1] * _q0 + dq[0] * _q1 + 
                    dq[3] * _q2 - dq[2] * _q3;
                const float tmpq2 = dq[2] * _q0 - dq[3] * _q1 + 
                    dq[0] * _q2 + dq[1] * _q3;
                const float tmpq3 = dq[3] * _q0 + dq[2] * _q1 - 
                    dq[1] * _q2 + dq[0] * _q3;

                // normalize and store the result
                float norm = device_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + 
                        tmpq3 * tmpq3) + EPSILON;
                _q0 = tmpq0 / norm;
                _q1 = tmpq1 / norm;
                _q2 = tmpq2 / norm;
                _q3 = tmpq3 / norm;
            }

            // Convert the new attitude to a rotation matrix, such that we can
            // rotate body-frame velocity and acc

            _r00 = _q0 * _q0 + _q1 * _q1 - _q2 * _q2 - _q3 * _q3;
            _r01 = 2 * _q1 * _q2 - 2 * _q0 * _q3;
            _r02 = 2 * _q1 * _q3 + 2 * _q0 * _q2;
            _r10 = 2 * _q1 * _q2 + 2 * _q0 * _q3;
            _r11 = _q0 * _q0 - _q1 * _q1 + _q2 * _q2 - _q3 * _q3;
            _r12 = 2 * _q2 * _q3 - 2 * _q0 * _q1;
            _r20 = 2 * _q1 * _q3 - 2 * _q0 * _q2;
            _r21 = 2 * _q2 * _q3 + 2 * _q0 * _q1;
            _r22 = _q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3;

            // reset the attitude error
            _x[STATE_D0] = 0;
            _x[STATE_D1] = 0;
            _x[STATE_D2] = 0;

            ekf_enforceSymmetry();

            _isUpdated = false;
        }


        bool isStateWithinBounds(void) 
        {
            for (int i = 0; i < 3; i++) {

                if (MAX_VELOCITY > 0.0f) {
                    if (_x[STATE_VX + i] > MAX_VELOCITY) {
                        return false;
                    } else if (_x[STATE_VX + i] < -MAX_VELOCITY) {
                        return false;
                    }
                }
            }

            return true;
        }

        void getVehicleState(vehicleState_t & state)
        {
            state.x = _x[STATE_X];

            state.dx = _r00*_x[STATE_VX] + 
                _r01*_x[STATE_VY] + 
                _r02*_x[STATE_VZ];

            state.y = _x[STATE_Y];

            // Negate for rightward positive
            state.dy = -(_r10*_x[STATE_VX] + 
                    _r11*_x[STATE_VY] + 
                    _r12*_x[STATE_VZ]);

            state.z = _x[STATE_Z];

            state.dz = _r20*_x[STATE_VX] + 
                _r21*_x[STATE_VY] + 
                _r22*_x[STATE_VZ];

            state.phi = RADIANS_TO_DEGREES *
                atan2f(2*(_q2*_q3+_q0*
                            _q1) ,
                        _q0*_q0 -
                        _q1*_q1 -
                        _q2*_q2 +
                        _q3*_q3);

            state.theta = RADIANS_TO_DEGREES * 
                asinf(-2*(_q1*_q3 -
                            _q0*_q2));

            state.psi = -RADIANS_TO_DEGREES *   // negate for nose-right positive
                atan2f(2*(_q1*_q2+_q0*
                            _q3)
                        , _q0*_q0 +
                        _q1*_q1 -
                        _q2*_q2 -
                        _q3*_q3);
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

        static constexpr float GRAVITY = 9.81;

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
        float _qinit0, _qinit1, _qinit2, _qinit3;

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
        float _r00, _r01, _r02, _r10, _r11, _r12, _r20, _r21, _r22; 

        // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
        // to allow easy normalization (in comparison to a rotation matrix),
        // while also being robust against singularities (in comparison to euler angles)
        float _q0, _q1, _q2, _q3;

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


            float dx_g = _x[STATE_VX];
            float dy_g = _x[STATE_VY];
            float z_g = 0.0;
            // Saturate elevation in prediction and correction to avoid singularities
            if ( _x[STATE_Z] < 0.1f ) {
                z_g = 0.1;
            } else {
                z_g = _x[STATE_Z];
            }

            // ~~~ X velocity prediction and update ~~~
            // predicts the number of accumulated pixels in the x-direction
            float hx[STATE_DIM] = {};
            _predictedNX = (flow->dt * Npix / thetapix ) * 
                ((dx_g * _r22 / z_g) - omegay_b);
            _measuredNX = flow->dpixelx*FLOW_RESOLUTION;

            // derive measurement equation with respect to dx (and z?)
            hx[STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dx_g) / (-z_g * z_g));
            hx[STATE_VX] = (Npix * flow->dt / thetapix) * 
                (_r22 / z_g);

            //First update
            ekf_updateWithScalar(hx, (_measuredNX-_predictedNX), 
                    flow->stdDevX*FLOW_RESOLUTION);

            // ~~~ Y velocity prediction and update ~~~
            float hy[STATE_DIM] = {};
            _predictedNY = (flow->dt * Npix / thetapix ) * 
                ((dy_g * _r22 / z_g) + omegax_b);
            _measuredNY = flow->dpixely*FLOW_RESOLUTION;

            // derive measurement equation with respect to dy (and z?)
            hy[STATE_Z] = (Npix * flow->dt / thetapix) * 
                ((_r22 * dy_g) / (-z_g * z_g));
            hy[STATE_VY] = (Npix * flow->dt / thetapix) * (_r22 / z_g);

            // Second update
            ekf_updateWithScalar(hy, (_measuredNY-_predictedNY),
                    flow->stdDevY*FLOW_RESOLUTION);

            _isUpdated = true;
        }

        void updateWithTof(tofMeasurement_t *tof)
        {
            // Updates the filter with a measured distance in the zb direction using the
            float h[STATE_DIM] = {};

            // Only update the filter if the measurement is reliable 
            // (\hat{h} -> infty when R[2][2] -> 0)
            if (fabs(_r22) > 0.1f && _r22 > 0) {
                float angle = 
                    fabsf(acosf(_r22)) - 
                    DEGREES_TO_RADIANS * (15.0f / 2.0f);
                if (angle < 0.0f) {
                    angle = 0.0f;
                }
                float predictedDistance = _x[STATE_Z] / cosf(angle);
                float measuredDistance = tof->distance; // [m]

                // This just acts like a gain for the sensor model. Further
                // updates are done in the scalar update function below
                h[STATE_Z] = 1 / cosf(angle); 

                ekf_updateWithScalar(h, measuredDistance-predictedDistance, tof->stdDev);

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

        // State vector
        __attribute__((aligned(4))) float _x[STATE_DIM];

        // Covariance matrix
        __attribute__((aligned(4))) float _p[STATE_DIM][STATE_DIM];

        // Covariance helper
        matrix_t _p_m;

        // Tracks whether an update to the state has been made, and the state
        // therefore requires finalization
        bool _isUpdated;

        uint32_t _lastPredictionMs;
        uint32_t _lastProcessNoiseUpdateMs;

        void ekf_init()
        {
            for (int i=0; i< STATE_DIM; i++) {

                _x[i] = 0;

                for (int j=0; j < STATE_DIM; j++) {
                    _p[i][j] = 0; 
                }
            }

            _p_m.numRows = STATE_DIM;
            _p_m.numCols = STATE_DIM;
            _p_m.pData = (float*)_p;
        }

        void ekf_addCovarianceNoise(const float * noise)
        {
            for (uint8_t k=0; k<STATE_DIM; ++k) {
                _p[k][k] += noise[k] * noise[k];
            }
        }

        void ekf_enforceSymmetry()
        {
            for (int i=0; i<STATE_DIM; i++) {

                for (int j=i; j<STATE_DIM; j++) {

                    ekf_pset(i, j, 0.5 * _p[i][j] + 0.5 * _p[j][i]);
                }
            }
        }

        // P_k = F_{k-1} P_{k-1} F^T_{k-1}
        void ekf_predict(const float F[STATE_DIM][STATE_DIM])
        {
            static __attribute__((aligned(4))) matrix_t Fm = { 
                STATE_DIM, STATE_DIM, (float *)F
            };

            static float tmpNN1d[STATE_DIM * STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN1m = { 
                STATE_DIM, STATE_DIM, tmpNN1d
            };

            static float tmpNN2d[STATE_DIM * STATE_DIM];
            static __attribute__((aligned(4))) matrix_t tmpNN2m = { 
                STATE_DIM, STATE_DIM, tmpNN2d
            };

            device_mat_mult(&Fm, &_p_m, &tmpNN1m); // F P
            device_mat_trans(&Fm, &tmpNN2m); // F'
            device_mat_mult(&tmpNN1m, &tmpNN2m, &_p_m); // F P F'

        }

        void ekf_updateWithScalar(const float * h, const float error, const float stdMeasNoise)
        {
            matrix_t Hm = {1, STATE_DIM, (float *)h};

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

            device_mat_trans(&Hm, &HTm);
            device_mat_mult(&_p_m, &HTm, &PHTm); // PH'
            float R = stdMeasNoise*stdMeasNoise;
            float HPHR = R; // HPH' + R
            for (int i=0; i<STATE_DIM; i++) { 
                // Add the element of HPH' to the above
                // this obviously only works if the update is scalar (as in this function)
                HPHR += Hm.pData[i]*PHTd[i]; 
            }

            // Calculate the Kalman gain and perform the state update
            for (int i=0; i<STATE_DIM; i++) {
                G[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                _x[i] = _x[i] + G[i] * error; // state update
            }

            device_mat_mult(&Gm, &Hm, &tmpNN1m); // GH
            for (int i=0; i<STATE_DIM; i++) { 
                tmpNN1d[STATE_DIM*i+i] -= 1; 
            } // GH - I
            device_mat_trans(&tmpNN1m, &tmpNN2m); // (GH - I)'
            device_mat_mult(&tmpNN1m, &_p_m, &tmpNN3m); // (GH - I)*P
            device_mat_mult(&tmpNN3m, &tmpNN2m, &_p_m); // (GH - I)*P*(GH - I)'

            // add the measurement variance and ensure boundedness and symmetry
            for (int i=0; i<STATE_DIM; i++) {

                for (int j=i; j<STATE_DIM; j++) {

                    float v = G[i] * R * G[j];

                    // add measurement noise
                    ekf_pset(i, j, 0.5 * _p[i][j] + 0.5 * _p[j][i] + v); 
                }
            }
        }

        void ekf_pset(const uint8_t i, const uint8_t j, const float pval)
        {
            if (isnan(pval) || pval > MAX_COVARIANCE) {
                _p[i][j] = _p[j][i] = MAX_COVARIANCE;
            } else if ( i==j && pval < MIN_COVARIANCE ) {
                _p[i][j] = _p[j][i] = MIN_COVARIANCE;
            } else {
                _p[i][j] = _p[j][i] = pval;
            }
        }
        void device_mat_trans(const matrix_t * pSrc, matrix_t * pDst); 

        void device_mat_mult(const matrix_t * pSrcA, const matrix_t * pSrcB,
                matrix_t * pDst);

        static float device_cos(const float x);

        static float device_sin(const float x);

        static float device_sqrt(const float32_t in);
};
