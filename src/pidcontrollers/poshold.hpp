/*
   poshold.hpp : PID-based position hold

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

#include "receiver.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {


    class PositionHold : public PID_Controller {

        friend class Hackflight;

        private:

        // Arbitrary constants
        const float WINDUP_MAX = 0.40f;

        // Setpoints for PID control
        Setpoint _setpointX;
        Setpoint _setpointY;

        bool gotCorrection(Setpoint & setpoint, float & demand, float position, float velocity, float currentTime) 
        {
            float correction = 0;
            if (setpoint.gotCorrection(demand, position, velocity, currentTime, correction)) {
                demand -= correction;
                return true;
            }

            return false;
        }

        protected:

        virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) 
        {
            bool didCorrectRoll  = gotCorrection(_setpointX, demands.roll,  state.positionY, state.velocityRightward, currentTime);
            bool didCorrectPitch = gotCorrection(_setpointY, demands.pitch, state.positionX, state.velocityForward,   currentTime);

            return didCorrectPitch || didCorrectRoll;
        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
        }

        public:

        PositionHold(float posP, float posrP, float posrI, float posrD=0.0f)
        {
            _setpointX.init(posP, posrP, posrI, posrD, WINDUP_MAX);
            _setpointY.init(posP, posrP, posrI, posrD, WINDUP_MAX);
        }

    };  // class PositionHold

} // namespace hf
