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

// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;

// millisecond time
typedef uint32_t timeMs_t ;

// microsecond time
typedef uint32_t timeUs_t;

#if defined(__cplusplus)
extern "C" {
#endif

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

void delayMicroseconds(timeUs_t us);
void delayMillis(timeMs_t ms);

timeUs_t microsISR(void);

timeUs_t timeMicros(void);
timeMs_t timeMillis(void);

uint32_t ticks(void);
timeDelta_t ticks_diff_us(uint32_t begin, uint32_t end);

#if defined(__cplusplus)
}
#endif
