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

#include <vector>
using namespace std;

#include "maxmotors.h"

class Esc {

    protected:

        vector<uint8_t> * m_pins;

        Esc(vector<uint8_t> & pins)
        {
            m_pins = &pins;
        }

    public:

        virtual void  begin(void) = 0;

        virtual float convertFromExternal(const uint16_t value) = 0;

        virtual float getMotorValue(
                const float input, const bool failsafeIsActive) = 0;

        virtual bool  isReady(const uint32_t currentTime) = 0;

        virtual void  stop(void) = 0;

        virtual void  write(const float values[]) = 0;
};
