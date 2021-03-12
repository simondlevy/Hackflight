/*
   Abstract class for closed-loop controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "state.hpp"

namespace hf {

    class ClosedLoopController {

        friend class ClosedLoopTask;

        protected:

            uint8_t modeIndex = 0;

            virtual bool shouldFlashLed(void)
            {
                return false;
            }

            virtual void resetOnInactivity(bool inactive)
            { 
                (void)inactive; 
            }

            virtual void modifyDemands(State * state, float * demands) = 0;

    };  // class ClosedLoopController

} // namespace hf
