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
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

    // Device functions
    void     gyroDevInit(void);
    uint32_t gyroDevInterruptCount(void);
    bool     gyroDevIsReady(void);
    int16_t  gyroDevReadRaw(uint8_t k);
    uint16_t gyroDevScaleDps(void);
    uint32_t gyroDevSyncTime(void);


#if defined(__cplusplus)
}
#endif
