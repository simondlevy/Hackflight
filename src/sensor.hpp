/*
   Abstract class for sensors

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "state.hpp"

namespace hf {

    class Sensor {

        friend class Hackflight;

        protected:

            virtual void modifyState(State * state, float time) = 0;

            virtual bool ready(float time) = 0;

            virtual void begin(void) { }

    };  // class Sensor

} // namespace hf
