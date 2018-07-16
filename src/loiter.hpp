/*
   loiter.hpp : PID-based loiter

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
#include "state.hpp"
#include "receiver.hpp"

namespace hf {


    class Loiter {

        friend class Hackflight;

        public:

        Loiter(float altitudeP, float altitudeD, float cyclicP, float throttleScale=1.f)
        {
            _altitudeP     = altitudeP;
            _altitudeD     = altitudeD;
            _cyclicP       = cyclicP;
            _throttleScale = throttleScale;

            _inBandPrev = false;
        }

        protected:

        virtual void modifyDemands(State & state, demands_t & demands) 
        {
            // Reset integral if moved into stick deadband
            bool inBandCurr = inBand(demands.throttle);
            if (inBandCurr && !_inBandPrev) {
                _altitudeTarget = state.altitude;
            }
            _inBandPrev = inBandCurr;

            // Throttle: inside stick deadband, adjust by variometer; outside deadband, respond weakly to stick demand
            demands.throttle = inBandCurr ?  
                _altitudeP * (_altitudeTarget-state.altitude) - _altitudeD * state.variometer: 
                _throttleScale*demands.throttle;

            // Pitch/roll
            demands.pitch = adjustCyclic(demands.pitch, state.velocityForward);
            demands.roll  = adjustCyclic(demands.roll,  state.velocityRightward);
        }

        bool inBand(float demand)
        {
            return fabs(demand) < Receiver::STICK_DEADBAND; 
        }

        float adjustCyclic(float demand, float velocity)
        {
            // Inside throttle deadband, adjust pitch/roll demand by PD controller; outside deadband, leave it as-is
            return inBand(demand) ? demand - _cyclicP*velocity: demand; 
        }

        float _altitudeTarget;
        float _altitudeP;
        float _altitudeD;
        float _cyclicP;
        float _throttleScale;
        bool  _inBandPrev;

    };  // class Loiter

} // namespace
