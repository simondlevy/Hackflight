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

            // Using digital pin 10 for chip select
            PMW3901 _flowSensor = PMW3901(10);

            float _flow[2];

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time; // XXX ignore time for now

                int16_t deltaX=0, deltaY=0;
                _flowSensor.readMotionCount(&deltaX, &deltaY);

                state.velocityForward   = (float)deltaX;
                state.velocityRightward = (float)deltaY;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return true; // XXX ignore readiness for now
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

            }

    };  // class OpticalFlow 

} // namespace hf
