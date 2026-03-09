/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <Arduino.h>

namespace hf {

    class LED {

        public:

            LED(const uint8_t pin) : _pin(pin) {}

            void begin()
            {
                pinMode(_pin, OUTPUT); 

                digitalWrite(_pin, HIGH);

                for (int j = 1; j<= 3; j++) {
                    digitalWrite(_pin, LOW);
                    delay(70);
                    digitalWrite(_pin, HIGH);
                    delay(360);
                }
            }

            void blink()
            {
                static uint32_t blink_counter, blink_delay;
                static bool blinkAlternate;

                const auto usec_curr = micros();

                if (usec_curr - blink_counter > blink_delay) {
                    blink_counter = micros();
                    digitalWrite(_pin, blinkAlternate); 

                    if (blinkAlternate == 1) {
                        blinkAlternate = 0;
                        blink_delay = 100000;
                    }
                    else if (blinkAlternate == 0) {
                        blinkAlternate = 1;
                        blink_delay = 2000000;
                    }
                }
            }

        private:

            uint8_t _pin;
    };
}
