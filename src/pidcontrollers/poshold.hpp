/*
   poshold.hpp : PID-based position hold

   Based on https://raw.githubusercontent.com/wiki/iNavFlight/inav/images/nav_poshold_pids_diagram.jpg

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

#include "debug.hpp"
#include "datatypes.hpp"
#include "receiver.hpp"
#include "pidcontroller.hpp"

namespace hf {


    class PositionHold : public PID_Controller {

        friend class Hackflight;

        public:

        PositionHold(float posP, float posrP, float posrI, float posrD=0.0f)
        {
            _posP = posP;
            _posrP = posrP;
            _posrI = posrI;
            _posrD = posrD;
        }

        protected:

        virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) 
        {
            (void)currentTime;

            demands.pitch = adjustCyclic(demands.pitch, state.velocityForward);
            demands.roll  = adjustCyclic(demands.roll,  state.velocityRightward);

            return true;
        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
        }

        private:

        // set by constructor
        float _posP;
        float _posrP;
        float _posrI;
        float _posrD;

        bool inBand(float demand)
        {
            return fabs(demand) < Receiver::STICK_DEADBAND; 
        }

        float adjustCyclic(float demand, float velocity)
        {
            // Inside throttle deadband, adjust pitch/roll demand by PD controller; outside deadband, leave it as-is
            return inBand(demand) ? demand - _posrP*velocity: demand; 
        }

    };  // class PositionHold

} // namespace hf
