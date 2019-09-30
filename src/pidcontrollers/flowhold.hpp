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

        friend class Hackflight;

        private: 

            // Arbitrary constants
            static constexpr float VEL_WINDUP_MAX   = 0.40f;
            static constexpr float PILOT_VELXY_MAX  = 2.5f; // http://ardupilot.org/copter/docs/altholdmode.html

            Pid _rollPid;

            bool _inBand = false;
            bool _inBandPrev = false;

        protected:

            void modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                // Is throttle stick in deadband?
                _inBand = fabs(demands.roll) < STICK_DEADBAND; 

                // Reset controller when moving into deadband
                if (_inBand && !_inBandPrev) {
                    _rollPid.reset();
                }
                _inBandPrev = _inBand;

                float targetVelocity = _inBand ? 0 : 2 * fabs(demands.roll) * PILOT_VELXY_MAX;

                debugline("vel: %+3.2f   tgt: %+3.2f", state.bodyVel[1], targetVelocity);

                // Run velocity PID controller to get correction
                //if (_inBand) demands.roll = _rollPid.compute(0, state.bodyVel[1], currentTime); 
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            FlowHoldPid(const float Kp, float Ki, float Kd)
            {
                _rollPid.init(Kp, Ki, Kd);

                _inBand = false;
                _inBandPrev = false;
            }

    };  // class FlowHoldPid

} // namespace hf
