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

#include "io_types.h"
#include "macros.h"

#define STATUS_LED_NUMBER 3

typedef struct statusLedConfig_s {
    ioTag_t ioTags[STATUS_LED_NUMBER];
    uint8_t inversion;
} statusLedConfig_t;

void ledInit(void);
void ledSet(bool state);
void ledToggle(void);
