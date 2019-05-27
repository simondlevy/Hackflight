/*
   flowhold.hpp : Positon-hold PID controller using optical flow (body-frame velocity)

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

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

#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class FlowHold : public PID_Controller {

        friend class Hackflight;

        private: 

        // PID constants set by constructor
        float _P = 0;
        float _minAltitude = 0;
        float _previousTime = 0;

        bool gotCorrection(float demand, float velocity, float currentTime, float & correction)
        {

            return false;
        }


        protected:

        bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
        {
            // Don't do anything till we've reached sufficient altitude
            if (state.location[2] < _minAltitude) return false;

            // Don't do anything until we have a positive deltaT
            float deltaT = currentTime - _previousTime;
            _previousTime = currentTime;
            if (deltaT == currentTime) return false;

            demands.roll  -= _P * state.inertialVel[1];
            demands.pitch -= _P * state.inertialVel[0];

            return false;

        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
        }

        public:

        FlowHold(float P, float minAltitude=0.1) : _P(P), _minAltitude(minAltitude)
        {
            _previousTime = 0;
        }

    };  // class FlowHold

} // namespace hf
