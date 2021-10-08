/*
   Abstract class for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_state.hpp"

namespace hf {

    class PidController {

        friend class HackflightPure;

        protected:

            virtual void modifyDemands(float * statevec, float * demands, bool ready) = 0;

    };  // class PidController

} // namespace hf
