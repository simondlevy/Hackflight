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

    static float getDt()
    {
        const auto usec_curr = micros();      
        static uint32_t _usec_prev;
        const float dt = (usec_curr - _usec_prev)/1000000.0;
        _usec_prev = usec_curr;

        return dt;
    }
}
