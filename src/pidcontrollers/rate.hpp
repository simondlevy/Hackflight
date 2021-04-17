/*
   Angular-velocity-based PID controller for roll, pitch, yaw

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

#include "receiver.hpp"
#include <RFT_filters.hpp>
#include "state.hpp"
#include "demands.hpp"
#include "pidcontroller.hpp"
#include "pidcontrollers/angvel.hpp"

namespace hf {

    class RatePid : public PidController {

        private: 

            // Rate mode uses a rate controller for roll, pitch
            AngularVelocityPid _rollPid;
            AngularVelocityPid _pitchPid;

        public:

            RatePid(const float Kp, const float Ki, const float Kd) 
            {
                _rollPid.begin(Kp, Ki, Kd);
                _pitchPid.begin(Kp, Ki, Kd);
            }

            void modifyDemands(State * state, float * demands)
            {
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL],  state->x[State::DPHI]);
                demands[DEMANDS_PITCH] = _pitchPid.compute(-demands[DEMANDS_PITCH], -state->x[State::DTHETA]);
            }

            virtual void updateReceiver(bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
                _rollPid.updateReceiver(throttleIsDown);
                _pitchPid.updateReceiver(throttleIsDown);
            }

    };  // class RatePid

} // namespace hf
