/*
   standard.hpp : Arduino code for brushless motor running on
   standard ESC

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

            // Min, max PWM values
            const uint16_t PWM_MIN = 1000;
            const uint16_t PWM_MAX = 2000;

        public:

            StandardMotor(uint8_t pin) 
                : Motor(pin)
            {
            }

            virtual void init(void) override
            {
                pinMode(_pin, OUTPUT);
                analogWrite(_pin, PWM_MIN>>3);
            }

            virtual void write(float value) override
            {
                analogWrite(_pin, (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)) >> 3);
            }

    }; // class StandardMotor

} // namespace hf
