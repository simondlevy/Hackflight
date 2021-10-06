/*
   Abstract class for sensors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_state.hpp"

namespace rft {

    class Sensor {

        friend class RFTPure;

        protected:

            virtual void modifyState(State * state, float time) = 0;

            virtual void begin(void) { }

    };  // class Sensor

} // namespace rft
