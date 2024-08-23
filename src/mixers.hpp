/*
   Motor mixers for Hackflight

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

            static void runBetaFlightQuadX(
                    const float t, const float r, const float p, const float y,
                    float & m1, float & m2, float & m3, float & m4)
            {
                m1 = t - r + p  - y;
                m2 = t - r - p  + y;
                m3 = t + r + p  + y;
                m4 = t + r - p  - y;
            }
     };

}
