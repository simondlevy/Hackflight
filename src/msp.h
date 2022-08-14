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

#include <stdbool.h>

#include "datatypes.h"
#include "receiver.h"

#if defined(__cplusplus)
extern "C" {
#endif

void mspInit(void);

void mspTriggerDebugging(void);

void mspUpdate(
        vehicle_state_t * state,
        Receiver::axes_t *rxaxes,
        bool armed,
        void * motorDevice,
        float * motors);

#if defined(__cplusplus)
}
#endif

class Msp {

    private:

        static const uint8_t MAX_PORT_COUNT = 3;
        static const uint8_t PORT_INBUF_SIZE = 192;
        static const uint16_t PORT_DATAFLASH_BUFFER_SIZE = 4096;
        static const uint8_t PORT_DATAFLASH_INFO_SIZE = 16;
        static const uint16_t PORT_OUTBUF_SIZE =
            PORT_DATAFLASH_BUFFER_SIZE + PORT_DATAFLASH_INFO_SIZE;
        static const uint8_t JUMBO_FRAME_SIZE_LIMIT = 255;

        static const uint8_t RC        = 105;
        static const uint8_t ATTITUDE  = 108;    
        static const uint8_t SET_MOTOR = 214;    


}; // class Msp

