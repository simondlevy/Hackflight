/*
   Abstract class for PID controllers

   Acts as a go-between for RFT state and Hackflight state

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_state.hpp"

namespace hf {

    class PidController {

        friend class PidTask;

        public:

            void modifyDemands(rft::State * state, float * demands)
            {
                modifyDemands(((State *)state)->x, demands);
            }

            virtual void modifyDemands(float * state, float * demands) = 0;

    };  // class PidController

} // namespace hf
