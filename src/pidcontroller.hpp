/*
   Abstract class for PID controllers, plus helper classes

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "state.hpp"
#include "demands.hpp"

#include <RFT_filters.hpp>

namespace hf {

    class PidController {

        friend class PidTask;

        protected:

            virtual void modifyDemands(State * state, float * demands) = 0;

            virtual bool shouldFlashLed(void) { return false; }

            virtual void resetOnInactivity(bool inactive) { (void)inactive; }

            uint8_t auxState = 0;

    };  // class PidController

} // namespace hf
