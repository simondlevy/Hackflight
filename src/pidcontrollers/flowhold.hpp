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

namespace hf {

    class FlowHoldPid : public PidController {

        public:

            FlowHoldPid(const float Kp, float Ki)
            {
                rollPid.begin(Kp, Ki, 0);
                pitchPid.begin(Kp, Ki, 0);
            }

        protected:

            virtual void modifyDemands(state_t * state, demands_t & demands) override
            {
                demands.pitch += (0.5 - fabs(demands.pitch)) * pitchPid.compute(0, state->inertialVel[0]);
                demands.roll  += (0.5 - fabs(demands.roll))  * rollPid.compute(0, state->inertialVel[1]);

            }

        private:

            Pid rollPid;
            Pid pitchPid;

    };  // class FlowHoldPid

} // namespace hf
