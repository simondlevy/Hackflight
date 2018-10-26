/*
   loiter.hpp : PID-based loiter

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


    class Loiter : public PID_Controller {

        friend class Hackflight;

        public:

        Loiter(float altitudeP, float altitudeD, float cyclicP, 
                float throttleScale=1.f, float minAltitude=0.5)
        {
            _altitudeP     = altitudeP;
            _altitudeD     = altitudeD;
            _cyclicP       = cyclicP;
            _throttleScale = throttleScale;
            _minAltitude   = minAltitude;

            _inBandPrev = false;
        }

        protected:

        virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) 
        {
            (void)currentTime;

            // Don't do anything till we've reached sufficient altitude
            if (state.altitude < _minAltitude) return false;

            // Reset altitude target if moved into stick deadband
            bool inBandCurr = inBand(demands.throttle);
            if (inBandCurr && !_inBandPrev) {
                _altitudeTarget = state.altitude;
            }
            _inBandPrev = inBandCurr;

            // Throttle: inside stick deadband, adjust by PID; outside deadband, respond to stick demand
            demands.throttle = inBandCurr ?  
                _altitudeP * (_altitudeTarget-state.altitude) - _altitudeD * state.variometer: 
                _throttleScale*demands.throttle;

            // Pitch/roll
            demands.pitch = adjustCyclic(demands.pitch, state.velocityForward);
            demands.roll  = adjustCyclic(demands.roll,  state.velocityRightward);

            return inBandCurr;
        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
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

        // set by constructor
        float _altitudeP;
        float _altitudeD;
        float _cyclicP;
        float _throttleScale;
        float _minAltitude;

        // modified in-flight
        float _altitudeTarget;
        bool  _inBandPrev;

    };  // class Loiter

} // namespace
