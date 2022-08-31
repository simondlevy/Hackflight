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

#define MAX_SUPPORTED_MOTORS 8

#if defined(__cplusplus)

class Esc {

    protected:

        uint8_t m_motorCount;

        Esc(uint8_t motorCount) 
        {
            m_motorCount = motorCount;
        }

    public:

        virtual void  begin(void) = 0;
        virtual float convertFromExternal(uint16_t value) = 0;
        virtual bool  isProtocolDshot(void) = 0;
        virtual bool  isReady(uint32_t currentTime) = 0;
        virtual float valueDisarmed(void) = 0;
        virtual float valueHigh(void) = 0;
        virtual float valueLow(void) = 0;
        virtual void  stop(void) = 0;
        virtual void  write(float *values) = 0;
};

#endif
