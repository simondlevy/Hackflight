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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"
#include "exti.h"
#include "io.h"

struct gyroDev_s;

typedef struct gyroDev_s {

    int16_t                   adcRaw[3];                          
    uint32_t                  detectedEXTI;
    volatile bool             dataReady;
    extDevice_t               dev;
    extiCallbackRec_t         exti;
    int32_t                   gyroDmaMaxDuration;
    uint32_t                  gyroLastEXTI;
    int32_t                   gyroShortPeriod;
    uint32_t                  gyroSyncEXTI;
    ioTag_t                   mpuIntExtiTag;

} gyroDev_t;

#if defined(__cplusplus)
extern "C" {
#endif

gyroDev_t * gyroContainerOf(extiCallbackRec_t * cb);

#if defined(__cplusplus)
}
#endif

