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

#include <stdlib.h>

#include "datatypes.h"
#include "debug.h"
#include "serial.h"
#include "time.h"

// Receive ISR callback
static void dsmxDataReceive(uint8_t c, void *data, uint32_t currentTimeUs)
{
    (void)c;
    (void)data;
    (void)currentTimeUs;
}

// Public API ==================================================================

uint8_t rxDevCheckDsmx(uint16_t * channelData, uint32_t * frameTimeUs)
{
    (void)channelData;
    (void)frameTimeUs;
    return 0;;
}

float rxDevConvertValueDsmx(uint16_t * channelData, uint8_t chan)
{
    (void)channelData;
    (void)chan;
    return 0;
}

void rxDevInitDsmx(serialPortIdentifier_e port)
{
    serialOpenPortDsmx(port, dsmxDataReceive);
}
