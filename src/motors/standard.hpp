/*
   Arduino code for brushless motor running on standard ESC

   Copyright (c) 2018 Juan Gallostra Acin, Simon D. Levy, Pep Martí Saumell

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

#include "motor.hpp"

namespace hf {

    class StandardMotor : public Motor {

        private:

#ifdef ESP32
            static const uint16_t OFFSET = 25;
#else
            static const uint16_t OFFSET = 0;
#endif
            static const uint16_t MINVAL = 125;
            static const uint16_t MAXVAL = 250;

            void writeValue(uint8_t index, uint16_t value)
            {
                analogWrite(_pins[index], value+OFFSET);
            }

        public:

            StandardMotor(const uint8_t pins[], const uint8_t count) 
                : Motor(pins, count)
            {
            }

            virtual void begin(void) override
            {
                for (uint8_t k=0; k<_count; ++k) {
                    pinMode(_pins[k], OUTPUT);
                    writeValue(k, MINVAL);
                }
            }

            virtual void write(uint8_t index, float value) override
            { 
                writeValue(index, (uint16_t)(MINVAL+value*(MAXVAL-MINVAL))); 
            }

    }; // class StandardMotor

} // namespace hf
