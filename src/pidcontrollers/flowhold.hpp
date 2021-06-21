/*
   Positon-hold PID controller using optical flow (body-frame velocity)

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class FlowHoldPid : public PidController {

        public:

            FlowHoldPid(const float Kp, float Ki)
            {
                rollPid.begin(Kp, Ki, 0);
                pitchPid.begin(Kp, Ki, 0);
            }

        protected:

            virtual void modifyDemands(state_t * state, demands_t & demands) override
            {
                demands.pitch += (0.5 - fabs(demands.pitch)) * pitchPid.compute(0, state->inertialVel[0]);
                demands.roll  += (0.5 - fabs(demands.roll))  * rollPid.compute(0, state->inertialVel[1]);

            }

        private:

            Pid rollPid;
            Pid pitchPid;

    };  // class FlowHoldPid

} // namespace hf
