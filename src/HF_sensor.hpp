/*
   Abstract class for sensors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_state.hpp"

namespace hf {

    class Sensor {

        public:

            virtual void modifyState(State * state, float time) = 0;

            virtual void begin(void) { }

    };  // class Sensor

} // namespace hf
