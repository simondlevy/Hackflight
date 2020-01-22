/*
   Superclass for Arduino-based flight controllers

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

#include "boards/realboard.hpp"

namespace hf {

    class ArduinoBoard : public RealBoard {

        private:

            uint8_t _led_pin = 0;
            bool    _led_inverted = false;

            static void powerPin(uint8_t id, uint8_t value)
            {
                pinMode(id, OUTPUT);
                digitalWrite(id, value);
            }

        protected:

            void setLed(bool isOn) 
            { 
                digitalWrite(_led_pin, isOn ?  (_led_inverted?LOW:HIGH) : (_led_inverted?HIGH:LOW));
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

            static void powerPins(uint8_t pwr, uint8_t gnd)
            {
                powerPin(pwr, HIGH);
                powerPin(gnd, LOW);
            }

            ArduinoBoard(uint8_t ledPin, bool ledInverted=false)
            {
                _led_pin = ledPin;
                _led_inverted = ledInverted;

                pinMode(_led_pin, OUTPUT);
                digitalWrite(_led_pin, _led_inverted ? HIGH : LOW);

                Serial.begin(115200);
                RealBoard::init();
            }

    }; // class ArduinoBoard

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
