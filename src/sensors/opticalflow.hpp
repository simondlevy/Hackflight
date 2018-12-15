/*
   opticalflow.hpp : Support for PMW3901 optical-flow sensor

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

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                int16_t deltaX=0, deltaY=0;
                _flowSensor.readMotionCount(&deltaX, &deltaY);

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
