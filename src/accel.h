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

#include "imu.h"

static const uint16_t ACCEL_1G   = 2048;
static const uint16_t ACCEL_RATE = 1000;

#if defined(__cplusplus)
extern "C" {
#endif

    void  accelInit(hackflight_t * hackflight);
    void  accelUpdate(void * hackflight, uint32_t usec);

    // Hardware-dependent
    bool  accelIsReady(void);
    float accelRead(uint8_t axis) ;

#if defined(__cplusplus)
}
#endif
