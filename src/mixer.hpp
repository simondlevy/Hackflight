/*
   Motor mixers interface for Hackflight

   Copyright (C) 2024 Simon D. Levy

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

#pragma once

#include <hackflight.hpp>

namespace hf {

    class Mixer {

        public:

            /**
              * Converts demands to motor values
              */
            virtual void run(const demands_t & demands, float * motors) = 0;

            virtual uint8_t rotorCount() = 0;

            /**
              * Returns positive (clockwise) or negative (counter-clockwise)
              * spin on motor for simulation dynamics
              */

            virtual int8_t roll(const uint8_t index) = 0;

            virtual int8_t pitch(const uint8_t index) = 0;

            virtual int8_t yaw(const uint8_t index) = 0;
     };

}
