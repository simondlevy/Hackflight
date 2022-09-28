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

#include "io_types.h"

typedef enum {
    BETAFLIGHT_EXTI_TRIGGER_RISING = 0,
    BETAFLIGHT_EXTI_TRIGGER_FALLING = 1,
    BETAFLIGHT_EXTI_TRIGGER_BOTH = 2
} extiTrigger_t;

typedef struct extiCallbackRec_s extiCallbackRec_t;

typedef void extiHandlerCallback();

struct extiCallbackRec_s {
    extiHandlerCallback *fn;
};

#if defined(__cplusplus)
extern "C" {
#endif

void extiInit(void);

void attachInterrupt(const uint8_t pin, extiHandlerCallback * isr);

#if defined(__cplusplus)
}
#endif
