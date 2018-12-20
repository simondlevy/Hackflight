/*
   lpf_opticalflow.hpp : Support for PMW3901 optical-flow sensor with low-pass filter


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

            static constexpr float   UPDATE_PERIOD = .01f;
            static const     uint8_t LPF_SIZE      = 64;

            // Use digital pin 10 for chip select
            PMW3901 _flowSensor = PMW3901(10);

            // Use low-pass filters for smoothing
            LowPassFilter _lpf_x = LowPassFilter(LPF_SIZE);
            LowPassFilter _lpf_y = LowPassFilter(LPF_SIZE);

            // Track elapsed time for periodic readiness
            float _previousTime;
            float _deltaTime;

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Avoid time blips
                if (_deltaTime > 0.02) return;

                // Read sensor
                int16_t dpixelx=0, dpixely=0;
                _flowSensor.readMotionCount(&dpixelx, &dpixely);

                // Scale readings by altitude, then low-pass filter them to get velocity
                state.velocityForward   =  _lpf_y.update(dpixely  * state.altitude * _deltaTime);
                state.velocityRightward =  _lpf_x.update(-dpixelx * state.altitude * _deltaTime);

                // Integrate velocity to get position
                state.positionX += state.velocityRightward;
                state.positionY += state.velocityForward;

                //Debug::printf("%+3.3f %+3.3f\n", state.positionX, state.positionY);
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

                _lpf_x.init();
                _lpf_y.init();

                _previousTime = 0;

            }

    };  // class OpticalFlow 

} // namespace hf
