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

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

    // For both hardware and sim implementations -----------------------------------------

    void gyroReadScaled(hackflight_t * hf, vehicle_state_t * vstate);

    // For hardware impelmentations ------------------------------------------------------

    void     gyroDevInit(void);
    void     gyroInit(hackflight_t * hf);
    uint32_t gyroInterruptCount(void);
    bool     gyroIsReady(void);
    int16_t  gyroReadRaw(uint8_t k);
    float    gyroScale(void);
    uint32_t gyroSyncTime(void);

#if defined(__cplusplus)
}
#endif

