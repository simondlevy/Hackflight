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

#include "esc.h"

#include <vector>

class BrushedEsc : public Esc {

    public:

        BrushedEsc(std::vector<uint8_t> & motorPins)  
            : Esc(BRUSHED, motorPins)
        {
        }

        virtual float  convertFromExternal(const uint16_t value) override 
        {
            return (value - 1000) / 1000.;
        }

        virtual float getMotorValue(const float input) override
        {
            return constrain_f(input, 0, 1);
        }

};
