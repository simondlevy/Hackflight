/*
   Butterfly dev board with USFS piggybacked

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include "rft_boards/realboards/arduino/butterfly.hpp"

namespace hf {

    class ButterflyPiggyback : public rft::Butterfly {

         public:

            void begin(void) override
            {
                rft::Butterfly::begin();

                // Use D4, D3 for power, gnd
                ArduinoBoard::powerPins(4, 3);

                // Hang a bit
                delay(100);
           
                // Start I^2C on D6, D7
                Wire.begin(TWI_PINS_6_7);

                // Hang a bit
                delay(100);
            }

    }; // class Butterfly

} // namespace hf
