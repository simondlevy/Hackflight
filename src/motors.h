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

#include "datatypes.h"
#include "time.h"

#define MAX_SUPPORTED_MOTORS 8

#if defined(__cplusplus)
extern "C" {
#endif

    float   motorDevConvertFromExternal(void * motorDevice, uint16_t externalValue);
    void    motorDevInitBrushed(uint8_t * pins);
    void  * motorDevInitDshot(uint8_t count);
    bool    motorDevIsProtocolDshot(void);
    bool    motorDevIsReady(uint32_t currentTime);
    float   motorDevValueDisarmed(void);
    float   motorDevValueHigh(void);
    float   motorDevValueLow(void);
    void    motorDevStop(void * motorDevice);
    void    motorDevWrite(void * motorDevice, float *values);

#if defined(__cplusplus)
}
#endif


#if defined(__cplusplus)

class Motors {

    public:

        float values[MAX_SUPPORTED_MOTORS];

        uint8_t m_count;

        Motors(float vals[], uint8_t count) {

            for (uint8_t k=0; k<count; ++k) {
                values[k] = vals[k];
            }

            m_count = count;
        }
        
}; // class Motors

#endif
