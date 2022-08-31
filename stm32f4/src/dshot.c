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
#include "dshot.h"
#include "dshot_command.h"
#include "escdev.h"
#include "nvic.h"
#include "pwm_output.h" // for ESC_* and others
#include "timer.h"
#include "dshot_dpwm.h" // for motorDmaOutput_t, should be gone

uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
        pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
    }

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum

    if (useDshotTelemetry) {
        csum = ~csum;
    }

    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}


FAST_DATA_ZERO_INIT dshotTelemetryState_t dshotTelemetryState;

uint16_t motorGetDshotTelemetry(uint8_t index)
{
    return dshotTelemetryState.motorState[index].telemetryValue;
}

FAST_DATA_ZERO_INIT dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];
