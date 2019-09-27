/*
   Positon-hold PID controller using optical flow (body-frame velocity)

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
#include "pid.hpp"

namespace hf {

    class FlowHoldPid : public PidController {

        friend class Hackflight;

        private: 

        Pid _rollPid;
        Pid _pitchPid;

        // PID constants set by constructor
        float _minAltitude = 0;

        protected:

        bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
        {
            // Don't do anything till we've reached sufficient altitude
            if (state.location[2] < _minAltitude) return false;

            demands.roll  = _rollPid.compute(0, state.bodyVel[1], currentTime);
            demands.pitch = _pitchPid.compute(0, state.bodyVel[0], currentTime);

            return true;

        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
        }

        public:

        FlowHoldPid(const float Kp, const float minAltitude=0.1) 
            : _minAltitude(minAltitude)
        {
            _rollPid.init(Kp, 0, 0);
            _pitchPid.init(Kp, 0, 0);
        }

    };  // class FlowHoldPid

} // namespace hf
