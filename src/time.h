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

#if defined(__cplusplus)
extern "C" {
#endif

    static inline int32_t cmpTimeUs(uint32_t a, uint32_t b)
    {
        return (int32_t)(a - b);
    }

    static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b)
    {
        return (int32_t)(a - b);
    }

    void delayMicroseconds(uint32_t us);
    void delayMillis(uint32_t ms);

    uint32_t microsISR(void);

    uint32_t timeMicros(void);
    uint32_t timeMillis(void);

    uint32_t ticks(void);
    int32_t ticks_diff_us(uint32_t begin, uint32_t end);

#if defined(__cplusplus)
}
#endif
