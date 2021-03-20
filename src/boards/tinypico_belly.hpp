/*
   TinyPICO dev board with USFS on bottom

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include "rft_boards/realboards/tinypico.hpp"

namespace hf {

    class TinyPicoBelly : public rft::TinyPico {

         public:

            void begin(void) override
            {
                rft::TinyPico::begin();

                // Use D18, D19 for power, gnd
                rft::ArduinoBoard::powerPins(18, 19);

                // Hang a bit
                delay(100);
           
                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class Butterfly

} // namespace hf
