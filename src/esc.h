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

#include "time.h"

#define MAX_SUPPORTED_MOTORS 8

#if defined(__cplusplus)
extern "C" {
#endif

    float   escDevConvertFromExternal(void * escDevice, uint16_t externalValue);
    void    escDevInitBrushed(uint8_t * pins);
    void  * escDevInitDshot(uint8_t count);
    bool    escDevIsProtocolDshot(void);
    bool    escDevIsReady(uint32_t currentTime);
    float   escDevValueDisarmed(void);
    float   escDevValueHigh(void);
    float   escDevValueLow(void);
    void    escDevStop(void * escDevice);
    void    escDevWrite(void * escDevice, float *values);

#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)

class Esc {

    protected:

        virtual float  convertFromExternal(void * escDevice, uint16_t externalValue) = 0;
        virtual void   initBrushed(uint8_t * pins) = 0;
        virtual void * initDshot(uint8_t count) = 0;
        virtual bool   isProtocolDshot(void) = 0;
        virtual bool   isReady(uint32_t currentTime) = 0;
        virtual float  valueDisarmed(void) = 0;
        virtual float  valueHigh(void) = 0;
        virtual float  valueLow(void) = 0;
        virtual void   stop(void * ice) = 0;
        virtual void   write(void * ice, float *values) = 0;

};

#endif
