/*
   Teensy 4.0 implementation of Hackflight Board routines

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"

namespace hf {

    class Teensy40 : public ArduinoBoard {

         public:

            Teensy40(void) 
                : ArduinoBoard(13)
            {
                // Start I^2C
                //Wire.begin(TWI_PINS_6_7);

                // Hang a bit
                delay(100);
            }

    }; // class Teensy40

} // namespace hf
