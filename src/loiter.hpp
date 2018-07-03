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

        Loiter(float varioP, float cyclicP, float throttleScale=0.10)
        {
            _varioP        = varioP;
            _cyclicP       = cyclicP;
            _throttleScale = throttleScale;
        }

        protected:

        void modifyDemands(State & state, demands_t & demands) 
        {
            // Throttle: inside stick deadband, adjust by variometer; inside deadband, respond weakly to stick demand
            demands.throttle = adjust(demands.throttle, _throttleScale*demands.throttle, 0, _varioP, state.variometer);     

            // Pitch/roll
            demands.pitch = adjustCyclic(demands.pitch, state.velocityForward);
            demands.roll  = adjustCyclic(demands.roll,  state.velocityRightward);
        }

        private:

        float adjustCyclic(float demand, float velocity)
        {
            // Inside stick deadband, adjust pitch/roll demand by velocity; outside deadband, leave it as-is
            return adjust(demand, demand, demand, _cyclicP, velocity);
        }

        static float adjust(float demand, float outBandValue, float baseValue, float P, float velocity)
        {
            return abs(demand) > Receiver::STICK_DEADBAND ? outBandValue : baseValue-P*velocity;
        }

        float _varioP;
        float _cyclicP;
        float _throttleScale;

    };  // class Loiter

} // namespace
