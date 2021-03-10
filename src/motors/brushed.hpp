/*
   Arduino code for brushed motors

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

#include "motor.hpp"

namespace hf {

    class BrushedMotor : public Motor {

        public:

            BrushedMotor(const uint8_t * pins, const uint8_t count) 
                : Motor(pins, count) 
            {
            }

            void begin(void) override
            {
                for (uint8_t k=0; k<_count; ++k) {
                    analogWriteFrequency(_pins[k], 10000);  
                    analogWrite(_pins[k], 0);  
                }
            }

            virtual void write(uint8_t index, float value) override
            {
                analogWrite(_pins[index], (uint8_t)(value * 255));
            }

    }; // class BrushedMotor

} // namespace hf
