/*
   Abstract class for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_state.hpp"

namespace hf {

    class PidController {

        friend class PidTask;

        public:

            void modifyDemands(State * state, float * demands)
            {
                modifyDemands(state->x, demands);
            }

            virtual void modifyDemands(float * statevec, float * demands) = 0;

    };  // class PidController

} // namespace hf
