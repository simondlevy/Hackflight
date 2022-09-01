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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <core/constrain.h>
#include <esc.h>
#include <pwm.h>
#include <time.h>

#include "platform.h"
#include "atomic.h"
#include "dshot_dev.h"
#include "dshot_command.h"
#include "esc_protocol.h"
#include "nvic.h"
#include "timer.h"

uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);

        // reset telemetry request to make sure it's triggered only once in a row
        pcb->requestTelemetry = false;    
    }

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum

    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}
