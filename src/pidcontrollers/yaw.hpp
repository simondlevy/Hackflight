/*
   Angular-velocity-based PID controller for yaw

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

#include "filters.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "angvel.hpp"

namespace hf {

    class YawPid : public PidController {

        private: 

            // Aribtrary constants
            static constexpr float BIG_YAW_DEMAND = 0.1f;

            // Rate mode uses a rate controller for roll, pitch
            _AngularVelocityPid _yawPid;

        public:

            YawPid(const float Kp, const float Ki) 
            {
                _yawPid.begin(Kp, Ki, 0);
            }

            void modifyDemands(state_t * state, demands_t & demands)
            {
                demands.yaw   = _yawPid.compute(demands.yaw, state->angularVel[2]);

                // Prevent "yaw jump" during correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // Reset yaw integral on large yaw command
                if (fabs(demands.yaw) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

            virtual void updateReceiver(bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
                _yawPid.updateReceiver(throttleIsDown);
            }

    };  // class RatePid

} // namespace hf
