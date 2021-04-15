/*
   TinyPICO dev board with I^2C

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include "rft_boards/realboards/tinypico.hpp"

namespace hf {

    class TinyPico : public rft::TinyPico {

         public:

            void begin(void) override
            {
                rft::TinyPico::begin();

                // Hang a bit
                delay(100);
           
                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class TinyPico

} // namespace hf
