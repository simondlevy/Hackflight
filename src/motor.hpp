/*
   Abstract parent class for running motors on Arduino

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

#ifdef ESP32
#include <analogWrite.h>
#endif

namespace hf {

    class NewMotor {

        private:

            static const uint8_t MAX_PINS = 20; // arbitrary

            uint8_t _pins[MAX_PINS];
            uint8_t _npins = 0;

        public:

            NewMotor(const uint8_t * pins, const uint8_t npins)
            {
                for (uint8_t k=0; k<npins; ++k) {
                    _pins[k] = pins[k];
                }
                _npins = npins;
            }

    }; // class NewMotor

    class Motor {

        protected:

            uint8_t _pin = 0;

            Motor(uint8_t pin)
            {
                _pin = pin;
            }

        public:

            virtual void write(float value)  = 0;

            virtual void init(void) { }

    }; // class Motor

} // namespace hf
