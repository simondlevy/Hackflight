/*
   ekf_opticalflow.hpp : Support for PMW3901 optical-flow sensor using Extended Kalman Filter

   State estimation adapted from:

https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/estimator_kalman.c

Copyright (c) 2018 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Hackflight is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>
#include <math.h>

#include <PMW3901.h>

#include "debugger.hpp"
#include "sensor.hpp"
#include "filters.hpp"
#include "linalg.hpp"

namespace hf {

    class OpticalFlow : public Sensor {

        private:

            static constexpr float UPDATE_PERIOD = .01f;
            static constexpr float FLOW_SCALE    = 100.f;

            // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
            static constexpr float MAX_COVARIANCE = 100.f;
            static constexpr float MIN_COVARIANCE = 1e-6f;
            static constexpr float MAX_POSITION   = 100.f; //meters
            static constexpr float MAX_VELOCITY   = 10.f;  //meters per second

            // Use digital pin 10 for chip select
            PMW3901 _flowSensor = PMW3901(10);

            // Track elapsed time for periodic readiness
            float _previousTime;

            // While tracking elapsed time, store delta time
            float _deltaTime;

            // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
            float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

            // The quad's state, stored as a column vector
            typedef enum
            {
                STATE_X,  // Position
                STATE_Y, 
                STATE_Z, 
                STATE_PX, // Velocity
                STATE_PY, 
                STATE_PZ, 
                STATE_D0, // Attitude error
                STATE_D1, 
                STATE_D2, 
                STATE_DIM
            } stateIdx_t;

            float S[STATE_DIM] = {0.f};

            float _omegax_b = 0;
            float _omegay_b = 0;
            float _dx_g = 0;
            float _dy_g = 0;
            float _z_g = 0;
            float _predictedNX = 0;
            float _predictedNY = 0;
            float _measuredNX = 0;
            float _measuredNY = 0;

            float q[4] = {1,0,0,0};

            Matrix Pm = Matrix(STATE_DIM, STATE_DIM);

            static constexpr float STDDEV = 0.25f;

            // ~~~ Camera constants ~~~
            // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
            float Npix = 30.0;                      // [pixels] (same in x and y)
            float thetapix = Filter::deg2rad(4.2f);

            uint32_t count = 0;

            static void checkNan(float x, const char * name, uint32_t count)
            {
                if (std::isnan(x)) {
                    Debugger::printf("%s is NaN after %d steps\n", name, count);
                    while (true) {
                    }
                }
            }

            void stateEstimatorAssertNotNaN() {

                for(int i=0; i<STATE_DIM; i++) {
                    if (std::isnan(S[i])) {
                        reset();
                        return;
                    }
                    for(int j=0; j<STATE_DIM; j++) {
                        if (std::isnan(Pm.get(i,j))) {
                            reset();
                            return;
                        }
                    }
                }
            }

            void reset(void)
            {
                for (uint8_t j=0; j<STATE_DIM; ++j) {
                    S[j] = 0;
                    for (uint8_t k=0; k<STATE_DIM; ++k) {
                        Pm.set(j,k,0);
                    }
                }
            }

            void stateEstimatorFinalize(void)
            {
                // Matrix to rotate the attitude covariances once updated
                static Matrix Am(STATE_DIM, STATE_DIM);

                // Temporary matrices for the covariance updates
                static Matrix tmpNN1m(STATE_DIM, STATE_DIM);
                static Matrix tmpNN2m(STATE_DIM, STATE_DIM);

                // Incorporate the attitude error (Kalman filter state) with the attitude
                float v0 = S[STATE_D0];
                float v1 = S[STATE_D1];
                float v2 = S[STATE_D2];

                // Move attitude error into attitude if any of the angle errors are large enough
                if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10)) {

                    float angle = sqrt(v0*v0 + v1*v1 + v2*v2);
                    float ca = cos(angle / 2.0f);
                    float sa = sin(angle / 2.0f);
                    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

                    // rotate the quad's attitude by the delta quaternion vector computed above
                    float tmpq0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
                    float tmpq1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
                    float tmpq2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
                    float tmpq3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

                    // normalize and store the result
                    float norm = sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
                    q[0] = tmpq0 / norm;
                    q[1] = tmpq1 / norm;
                    q[2] = tmpq2 / norm;
                    q[3] = tmpq3 / norm;

                    /** Rotate the covariance, since we've rotated the body
                     *
                     * This comes from a second order approximation to:
                     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
                     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
                     * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
                     *
                     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
                     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
                     */

                    float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
                    float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
                    float d2 = v2/2;

                    Am.set(STATE_X,STATE_X, 1);
                    Am.set(STATE_Y,STATE_Y, 1);
                    Am.set(STATE_Z,STATE_Z, 1);

                    Am.set(STATE_PX,STATE_PX, 1);
                    Am.set(STATE_PY,STATE_PY, 1);
                    Am.set(STATE_PZ,STATE_PZ, 1);

                    Am.set(STATE_D0,STATE_D0,  1 - d1*d1/2 - d2*d2/2);
                    Am.set(STATE_D0,STATE_D1,  d2 + d0*d1/2);
                    Am.set(STATE_D0,STATE_D2, -d1 + d0*d2/2);

                    Am.set(STATE_D1,STATE_D0, -d2 + d0*d1/2);
                    Am.set(STATE_D1,STATE_D1,  1 - d0*d0/2 - d2*d2/2);
                    Am.set(STATE_D1,STATE_D2,  d0 + d1*d2/2);

                    Am.set(STATE_D2,STATE_D0,  d1 + d0*d2/2);
                    Am.set(STATE_D2,STATE_D1, -d0 + d1*d2/2);
                    Am.set(STATE_D2,STATE_D2, 1 - d0*d0/2 - d1*d1/2);

                    Matrix::trans(Am, tmpNN1m); // A'
                    Matrix::mult(Am, Pm, tmpNN2m); // AP
                    Matrix::mult(tmpNN2m, tmpNN1m, Pm); //APA'
                }

                // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
                R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
                R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
                R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

                R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
                R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
                R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

                R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
                R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
                R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

                // reset the attitude error
                S[STATE_D0] = 0;
                S[STATE_D1] = 0;
                S[STATE_D2] = 0;

                // constrain the states
                for (int i=0; i<3; i++)
                {
                    if (S[STATE_X+i] < -MAX_POSITION) { S[STATE_X+i] = -MAX_POSITION; }
                    else if (S[STATE_X+i] > MAX_POSITION) { S[STATE_X+i] = MAX_POSITION; }

                    if (S[STATE_PX+i] < -MAX_VELOCITY) { S[STATE_PX+i] = -MAX_VELOCITY; }
                    else if (S[STATE_PX+i] > MAX_VELOCITY) { S[STATE_PX+i] = MAX_VELOCITY; }
                }

                // enforce symmetry of the covariance matrix, and ensure the values stay bounded
                for (int i=0; i<STATE_DIM; i++) {
                    for (int j=i; j<STATE_DIM; j++) {
                        float p = 0.5f*Pm.get(i,j) + 0.5f*Pm.get(j,i);
                        if (std::isnan(p) || p > MAX_COVARIANCE) {
                            Pm.set(i, j, MAX_COVARIANCE);
                            Pm.set(j, i, MAX_COVARIANCE);
                        } else if ( i==j && p < MIN_COVARIANCE ) {
                            Pm.set(i, j, MIN_COVARIANCE);
                            Pm.set(j, i, MIN_COVARIANCE);
                        } else {
                            Pm.set(i, j, p);
                            Pm.set(j, i, p);
                        }
                    }
                }
            }

            void stateEstimatorScalarUpdate(Matrix & Hm, float error, float stdMeasNoise, const char * label)
            {
                // The Kalman gain as a column vector
                static Matrix Km(STATE_DIM, 1);

                // Temporary matrices for the covariance updates
                static Matrix tmpNN1m(STATE_DIM, STATE_DIM);
                static Matrix tmpNN2m(STATE_DIM, STATE_DIM);
                static Matrix tmpNN3m(STATE_DIM, STATE_DIM);
                static Matrix HTm(STATE_DIM, 1);
                static Matrix PHTm(STATE_DIM, 1);

                // ====== INNOVATION COVARIANCE ======

                Matrix::trans(Hm, HTm);

                Matrix::mult(Pm, HTm, PHTm); // PH'
                float R = stdMeasNoise*stdMeasNoise;
                float HPHR = R; // HPH' + R


                for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
                    HPHR += Hm.get(0,i)*PHTm.get(i,0); // this obviously only works if the update is scalar (as in this function)
                }

                checkNan(HPHR, "HPHR", count++);

                // ====== MEASUREMENT UPDATE ======
                // Calculate the Kalman gain and perform the state update
                for (int i=0; i<STATE_DIM; i++) {
                    Km.set(i,0, PHTm.get(i,0)/HPHR); // kalman gain = (PH' (HPH' + R )^-1)
                    S[i] += Km.get(i,0) * error; // state update
                }
                stateEstimatorAssertNotNaN();

                // ====== COVARIANCE UPDATE ======
                Matrix::mult(Km, Hm, tmpNN1m); // KH
                for (int i=0; i<STATE_DIM; i++) { 
                    tmpNN1m.set(i,i, tmpNN1m.get(i,i)-1);// KH - I
                }
                Matrix::trans(tmpNN1m, tmpNN2m); // (KH - I)'
                Matrix::mult(tmpNN1m, Pm, tmpNN3m); // (KH - I)*P
                Matrix::mult(tmpNN3m, tmpNN2m, Pm); // (KH - I)*P*(KH - I)'

                //stateEstimatorAssertNotNaN();
                // add the measurement variance and ensure boundedness and symmetry
                // TODO: Why would it hit these bounds? Needs to be investigated.
                for (int i=0; i<STATE_DIM; i++) {
                    for (int j=i; j<STATE_DIM; j++) {
                        float v = Km.get(i,0) * R * Km.get(j,0);
                        float p = 0.5f*Pm.get(i,j) + 0.5f*Pm.get(j,i) + v; // add measurement noise
                        if (std::isnan(p) || p > MAX_COVARIANCE) {
                            Pm.set(i,j, MAX_COVARIANCE);
                            Pm.set(j,i, MAX_COVARIANCE);
                        } else if ( i==j && p < MIN_COVARIANCE ) {
                            Pm.set(i,j, MIN_COVARIANCE);
                            Pm.set(j,i, MIN_COVARIANCE);
                        } else {
                            Pm.set(i,j, p);
                            Pm.set(j,i, p);
                        }
                    }
                }

                stateEstimatorAssertNotNaN();
            }


        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Avoid time blips
                if (_deltaTime > 0.02) return;

                S[STATE_Z] = state.altitude;
                S[STATE_PZ] = state.variometer;

                // Read the flow sensor
                int16_t dpixelx=0, dpixely=0;
                _flowSensor.readMotionCount(&dpixelx, &dpixely);

                //~~~ Body rates ~~~
                // TODO check if this is feasible or if some filtering has to be done
                _omegax_b = state.angularVelocities[0];
                _omegay_b = state.angularVelocities[1];

                _dx_g = S[STATE_PX];
                _dy_g = S[STATE_PY];

                // Saturate elevation in prediction and correction to avoid singularities
                _z_g = S[STATE_Z] < 0.1f ?  0.1f : S[STATE_Z];

                // ~~~ X velocity prediction and update ~~~
                // predicts the number of accumulated pixels in the x-direction
                float omegaFactor = 1.25f;
                Matrix Hx(1, STATE_DIM);
                _predictedNX = (_deltaTime * Npix / thetapix ) * ((_dx_g * R[2][2] / _z_g) - omegaFactor * _omegay_b);
                _measuredNX = (float)dpixelx * FLOW_SCALE;

                // derive measurement equation with respect to dx (and z?)
                Hx.set(0, STATE_Z,  (Npix * _deltaTime / thetapix) * ((R[2][2] * _dx_g) / (-_z_g * _z_g)));
                Hx.set(0, STATE_PX, (Npix * _deltaTime / thetapix) * (R[2][2] / _z_g));

                //First update
                stateEstimatorScalarUpdate(Hx, _measuredNX-_predictedNX, STDDEV, "X");

                // ~~~ Y velocity prediction and update ~~~
                Matrix Hy(1, STATE_DIM);
                _predictedNY = (_deltaTime * Npix / thetapix ) * ((_dy_g * R[2][2] / _z_g) + omegaFactor * _omegax_b);
                _measuredNY = (float)dpixely * FLOW_SCALE;

                // derive measurement equation with respect to dy (and z?)
                Hy.set(0, STATE_Z,  (Npix * _deltaTime / thetapix) * ((R[2][2] * _dy_g) / (-_z_g * _z_g)));
                Hy.set(0, STATE_PY, (Npix * _deltaTime / thetapix) * (R[2][2] / _z_g));

                // Second update
                stateEstimatorScalarUpdate(Hy, _measuredNY-_predictedNY, STDDEV, "Y");

                stateEstimatorFinalize();

                Debugger::printf("%+3.3f,%+3.3f\n", S[STATE_PX], S[STATE_PY]);

                state.velocityForward   = 0;
                state.velocityRightward = 0;
            }

            virtual bool ready(float time) override
            {
                _deltaTime = time - _previousTime; 

                bool result = _deltaTime > UPDATE_PERIOD;

                if (result) {

                    _previousTime = time;
                }

                return result;
            }

        public:

            void begin(void)
            {
                if (!_flowSensor.begin()) {
                    while (true) {
                        Serial.println("Initialization of the flow sensor failed");
                        delay(500);
                    }
                }

                _previousTime = 0;

            }

    };  // class OpticalFlow 

} // namespace hf
