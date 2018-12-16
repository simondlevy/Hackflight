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
