/*
   Yaw PID controller

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
#include "filters.hpp"
#include "datatypes.hpp"
#include "rate.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class YawPid : public PidController {

        friend class Hackflight;

        private: 

        // Arbitrary
        const float BIG_YAW_DEMAND = 0.1f;

        RatePid _ratePid;

        public:

        YawPid(float P, float I, float demandScale = 1.0f, float D=0.0f) 
        {
            // PI controller
            _ratePid.init(P, I, D, demandScale);
        }

        bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
        {
            (void)currentTime;

            float itermFactor = 1.0;

            // Reset integral on large yaw command
            if (fabs(demands.yaw) > BIG_YAW_DEMAND) {
                itermFactor = 0.0;
                _ratePid.resetIntegral();
            }

            demands.yaw = _ratePid.compute(demands.yaw, state.angularVel[2], itermFactor);

            // Prevent "yaw jump" during gyroYawPid correction
            demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

            // We've always gotta do this!
            return true;
        }

        virtual void updateReceiver(demands_t & demands, bool throttleIsDown) override
        {
            _ratePid.updateReceiver(demands, throttleIsDown);
        }

    };  // class YawPid

} // namespace hf
