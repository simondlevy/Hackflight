/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "utils.h"

class Motors {

    public:

        static const uint8_t MAX_SUPPORTED = 8;

        float values[MAX_SUPPORTED];

        uint8_t m_count;

        Motors(const float vals[], const uint8_t count) {

            for (auto k=0; k<count; ++k) {
                values[k] = vals[k];
            }

            m_count = count;
        }
        
}; // class Motors
