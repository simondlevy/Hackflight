/*
   Ladybug dev board implementation of Hackflight Board routines

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"

namespace hf {

    class Ladybug : public ArduinoBoard {

        public:

            Ladybug(void) 
                : ArduinoBoard(13)
            {
                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class Ladybug

} // namespace hf
