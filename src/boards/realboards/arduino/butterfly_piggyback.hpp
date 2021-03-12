/*
   Butterfly implementation of Hackflight Board routines

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"

namespace hf {

    class Butterfly : public ArduinoBoard {

         public:

            Butterfly(void) 
                : ArduinoBoard(13, true) // red LED, active low
            {
            }

            void begin(void) override
            {
                ArduinoBoard::begin();

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
