/*
   Butterfly implementation of Hackflight Board routines

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
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
