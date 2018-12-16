/*
   opticalflow.hpp : Support for PMW3901 optical-flow sensor

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

#include "debug.hpp"
#include "peripheral.hpp"
#include "filters.hpp"
#include "linalg.hpp"

namespace hf {

    class OpticalFlow : public PeripheralSensor {

        private:

            static constexpr float UPDATE_PERIOD = .01f;

            // Use digital pin 10 for chip select
            PMW3901 _flowSensor = PMW3901(10);

            // Track elapsed time for periodic readiness
            float _previousTime;

            // While tracking elapsed time, store delta time
            float _deltaTime;

            // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
            static constexpr float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
           
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

            static constexpr float STDDEV = 0.25f;

            // ~~~ Camera constants ~~~
            // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
            float Npix = 30.0;                      // [pixels] (same in x and y)
            float thetapix = Filter::deg2rad(4.2f);

            void stateEstimatorScalarUpdate(float *Hm, float error, float stdMeasNoise)
            {
                // The Kalman gain as a column vector
                static float K[STATE_DIM];
                //static arm_matrix_instance_f32 Km = {STATE_DIM, 1, (float *)K};

                // Temporary matrices for the covariance updates
                static float tmpNN1d[STATE_DIM * STATE_DIM];
                //static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

                static float tmpNN2d[STATE_DIM * STATE_DIM];
                //static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

                static float tmpNN3d[STATE_DIM * STATE_DIM];
                //static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

                static float HTd[STATE_DIM * 1];
                //static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

                static float PHTd[STATE_DIM * 1];
                //static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};

                //configASSERT(Hm->numRows == 1);
                //configASSERT(Hm->numCols == STATE_DIM);

                /*

                // ====== INNOVATION COVARIANCE ======

                mat_trans(Hm, &HTm);
                mat_mult(&Pm, &HTm, &PHTm); // PH'
                float R = stdMeasNoise*stdMeasNoise;
                float HPHR = R; // HPH' + R
                for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
                    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
                }
                configASSERT(!isnan(HPHR));

                // ====== MEASUREMENT UPDATE ======
                // Calculate the Kalman gain and perform the state update
                for (int i=0; i<STATE_DIM; i++) {
                    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                    S[i] = S[i] + K[i] * error; // state update
                }
                stateEstimatorAssertNotNaN();

                // ====== COVARIANCE UPDATE ======
                mat_mult(&Km, Hm, &tmpNN1m); // KH
                for (int i=0; i<STATE_DIM; i++) { tmpNN1d[STATE_DIM*i+i] -= 1; } // KH - I
                mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
                mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
                mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
                stateEstimatorAssertNotNaN();
                // add the measurement variance and ensure boundedness and symmetry
                // TODO: Why would it hit these bounds? Needs to be investigated.
                for (int i=0; i<STATE_DIM; i++) {
                    for (int j=i; j<STATE_DIM; j++) {
                        float v = K[i] * R * K[j];
                        float p = 0.5f*P[i][j] + 0.5f*P[j][i] + v; // add measurement noise
                        if (isnan(p) || p > MAX_COVARIANCE) {
                            P[i][j] = P[j][i] = MAX_COVARIANCE;
                        } else if ( i==j && p < MIN_COVARIANCE ) {
                            P[i][j] = P[j][i] = MIN_COVARIANCE;
                        } else {
                            P[i][j] = P[j][i] = p;
                        }
                    }
                }

                stateEstimatorAssertNotNaN();
                */
            }


        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                S[STATE_Z] = state.altitude;
                S[STATE_PZ] = state.variometer;

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
                float hx[STATE_DIM] = {0};
                Matrix Hx(1, STATE_DIM, hx);
                //arm_matrix_instance_f32 Hx = {1, STATE_DIM, hx};
                _predictedNX = (_deltaTime * Npix / thetapix ) * ((_dx_g * R[2][2] / _z_g) - omegaFactor * _omegay_b);
                _measuredNX = (float)dpixelx;

                // derive measurement equation with respect to dx (and z?)
                hx[STATE_Z] = (Npix * _deltaTime / thetapix) * ((R[2][2] * _dx_g) / (-_z_g * _z_g));
                hx[STATE_PX] = (Npix * _deltaTime / thetapix) * (R[2][2] / _z_g);

                //First update
                //stateEstimatorScalarUpdate(&Hx, measuredNX-predictedNX, STDDEV);

                // ~~~ Y velocity prediction and update ~~~
                float hy[STATE_DIM] = {0};
                //arm_matrix_instance_f32 Hy = {1, STATE_DIM, hy};
                _predictedNY = (_deltaTime * Npix / thetapix ) * ((_dy_g * R[2][2] / _z_g) + omegaFactor * _omegax_b);
                _measuredNY = (float)dpixely;

                // derive measurement equation with respect to dy (and z?)
                hy[STATE_Z] = (Npix * _deltaTime / thetapix) * ((R[2][2] * _dy_g) / (-_z_g * _z_g));
                hy[STATE_PY] = (Npix * _deltaTime / thetapix) * (R[2][2] / _z_g);

                // Second update
                //stateEstimatorScalarUpdate(&Hy, measuredNY-predictedNY, STDDEV);

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
