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

            float _omegax_b;
            float _omegay_b;
            float _dx_g;
            float _dy_g;
            float _z_g;
            float _predictedNX;
            float _predictedNY;
            float _measuredNX;
            float _measuredNY;

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                int16_t deltaX=0, deltaY=0;
                _flowSensor.readMotionCount(&deltaX, &deltaY);

                // ~~~ Camera constants ~~~
                // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
                float Npix = 30.0;                      // [pixels] (same in x and y)
                //float thetapix = DEG_TO_RAD * 4.2f;
                //~~~ Body rates ~~~
                // TODO check if this is feasible or if some filtering has to be done
                //_omegax_b = sensors->gyro.x * DEG_TO_RAD;
                //_omegay_b = sensors->gyro.y * DEG_TO_RAD;

                // ~~~ Moves the body velocity into the global coordinate system ~~~
                // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
                //
                // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
                // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
                //
                // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
                // _G and _B refer to the global/body coordinate systems.

                // Modification 1
                //dx_g = R[0][0] * S[STATE_PX] + R[0][1] * S[STATE_PY] + R[0][2] * S[STATE_PZ];
                //dy_g = R[1][0] * S[STATE_PX] + R[1][1] * S[STATE_PY] + R[1][2] * S[STATE_PZ];

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
