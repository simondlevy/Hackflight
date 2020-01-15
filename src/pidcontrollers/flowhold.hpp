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

            // Helper class
            class _FlowVelocityPid : public VelocityPid {

                private:

                    // Arbitrary constants
                    static constexpr float PILOT_VELXY_MAX  = 2.5f; // http://ardupilot.org/copter/docs/altholdmode.html

                public:

                    void init(float Kp, float Ki)
                    {
                        VelocityPid::init(Kp, Ki, 0);
                    }

                    void update(float & demand, float velocity)
                    {
                        demand = VelocityPid::compute(demand, 0, 2*PILOT_VELXY_MAX, velocity);
                    }

            }; // _FlowVelocityPid

            _FlowVelocityPid _rollPid;
            _FlowVelocityPid _pitchPid;

        protected:

            void modifyDemands(state_t * state, demands_t * demands)
            {
                _rollPid.update(demands->roll,  state->bodyVel[1]);
                _rollPid.update(demands->pitch, state->bodyVel[0]);
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            FlowHoldPid(const float Kp, float Ki)
            {
                _rollPid.init(Kp, Ki);
                _pitchPid.init(Kp, Ki);
            }

    };  // class FlowHoldPid

} // namespace hf
