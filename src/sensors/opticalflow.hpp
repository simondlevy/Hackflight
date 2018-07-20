/*
   opticalflow.hpp : Support for optical-flow sensors

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

#include "debug.hpp"
#include "sensor.hpp"

namespace hf {

    class OpticalFlow : public Sensor {

        private:

            float _flow[2];

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time; // XXX ignore time for now

                state.velocityForward   = _flow[0];
                state.velocityRightward = _flow[1];
            }

            virtual bool ready(float time) override
            {
                (void)time;

                getFlow(_flow);

                return true; // XXX ignore readiness for now
            }

            virtual void getFlow(float flow[2]) = 0;

    };  // class OpticalFlow

} // namespace
