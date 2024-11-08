/*
   Complementary filter for altitude/climb-rate sensor fusion

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
#include <utils.hpp>

namespace hf {

    class ComplementaryVertical {

        public:

            void getValues(
                    const float dt,
                    const axis3_t & accel,
                    const float zrange,
                    const axis3_t & angles,
                    float & z,float & dz)
            {
                (void)dt;
                (void)accel;
                (void)zrange;
                (void)angles;
                (void)z;
                (void)dz;
            }
    };

}
