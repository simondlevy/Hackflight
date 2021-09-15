/*
   Abstract class for PID controllers

   Acts as a go-between for hf state and Hackflight state

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "state.hpp"

namespace hf {

    class PidController {

        friend class PidControlTask;

        protected:

            uint8_t modeIndex = 0;

            virtual void modifyDemands(float * state, float * demands) = 0;

            virtual void resetOnInactivity(bool inactive)
            { 
                (void)inactive; 
            }

    };  // class PidController

} // namespace hf
