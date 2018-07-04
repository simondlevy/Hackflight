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

        Loiter(float varioP, float varioI, float cyclicP, float throttleScale=0.10)
        {
            _varioP        = varioP;
            _varioI        = varioI;
            _cyclicP       = cyclicP;
            _throttleScale = throttleScale;

            _varioIntegral = 0;
            _inBandPrev = false;
        }

        protected:

        void modifyDemands(State & state, demands_t & demands) 
        {
            Debug::printf("var: %+6.6f  varint: %+6.6f  for: %+4.4f   rgt: %+4.4f\n", 
                    state.variometer, _varioIntegral, state.velocityForward, state.velocityRightward);

            // Reset integral if moved into stick deadband
            bool inBandCurr = inBand(demands.throttle);
            if (inBandCurr && !_inBandPrev) {
                _varioIntegral = 0;
            }
            _inBandPrev = inBandCurr;


            // Throttle: inside stick deadband, adjust by variometer; outside deadband, respond weakly to stick demand
            demands.throttle = inBandCurr ?  -_varioP*state.variometer - _varioI*_varioIntegral: _throttleScale*demands.throttle;

            // Pitch/roll
            demands.pitch = adjustCyclic(demands.pitch, state.velocityForward);
            demands.roll  = adjustCyclic(demands.roll,  state.velocityRightward);

            // Accumulate integrals
            _varioIntegral += state.variometer;
        }

        private:

        float adjustCyclic(float demand, float velocity)
        {
            // Inside stick deadband, adjust pitch/roll demand by velocity; outside deadband, leave it as-is
            return inBand(demand) ? demand - _cyclicP*velocity: demand; 
        }

        bool inBand(float demand)
        {
            return abs(demand) < Receiver::STICK_DEADBAND; 
        }

        float _varioI;
        float _varioP;
        float _cyclicP;
        float _throttleScale;

        float _varioIntegral;
        bool  _inBandPrev;

    };  // class Loiter

} // namespace
