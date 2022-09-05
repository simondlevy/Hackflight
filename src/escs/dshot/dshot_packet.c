/*
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

#if !defined(ARDUINO)

#include "platform.h"
#include "atomic.h"
#include "escs/dshot/dshot_dev.h"
#include "nvic.h"

uint16_t getDshotPacketAtomic(dshotProtocolControl_t *pcb)
{
    uint16_t packet = 0;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);

        // reset telemetry request to make sure it's triggered only once in a row
        pcb->requestTelemetry = false;    
    }

    return packet;
}

#endif
