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

    class Motor {

        protected:

            static const uint8_t MAX_COUNT = 20; // arbitrary

            uint8_t _pins[MAX_COUNT];
            uint8_t _count = 0;

            Motor(const uint8_t count) 
            {
                _count = count;
            }

            Motor(const uint8_t * pins, const uint8_t count)
            {
                for (uint8_t k=0; k<count; ++k) {
                    _pins[k] = pins[k];
                }
                _count = count;
            }

        public:

            virtual void init(void) { }

            virtual void write(uint8_t index, float value) = 0;

    }; // class Motor

} // namespace hf
