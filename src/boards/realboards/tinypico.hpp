/*
   TinyPICO implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2019 Simon D. Levy

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
#include "motors/standard.hpp"
#include "boards/realboard.hpp"
#include "boards/realboards/arduino.hpp"

#include <TinyPICO.h>

namespace hf {

    class TinyPico : public RealBoard {

        private:

            TinyPICO tp;

        protected:

            void setLed(bool isOn) 
            { 
                tp.DotStar_SetPixelColor(0, isOn?255:0, 0);
            }

            uint8_t serialNormalAvailable(void)
            {
                return Serial.available();
            }

            uint8_t serialNormalRead(void)
            {
                return Serial.read();
            }

            void serialNormalWrite(uint8_t c)
            {
                Serial.write(c);
            }

         public:

            TinyPico(void) 
            {
                Serial.begin(115200);

                // This will blink the LED
                RealBoard::init();

                // Use D18,19 for SENtral power, ground
                ArduinoBoard::powerPins(18, 19);

                // Hang a bit 
                delay(100);

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class TinyPico

} // namespace hf
