/*
   Abstract class for PID controllers

   Acts as a go-between for RFT state and Hackflight state

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_closedloop.hpp>

#include "HF_state.hpp"

namespace hf {

    class PidController : public rft::ClosedLoopController {

        friend class PidTask;

        protected:

            virtual void modifyDemands(rft::State * state, float * demands) override
            {
                modifyDemands(((State *)state)->x, demands);
            }

            virtual void modifyDemands(float * state, float * demands) = 0;

    };  // class PidController

} // namespace hf
