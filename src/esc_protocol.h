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

#include <esc.h>
#include <time.h>

typedef enum {
    ESC_STANDARD = 0,
    ESC_ONESHOT125,
    ESC_ONESHOT42,
    ESC_MULTISHOT,
    ESC_BRUSHED,
    ESC_DSHOT150,
    ESC_DSHOT300,
    ESC_DSHOT600,
    ESC_PROSHOT1000,
    ESC_DISABLED,
    ESC_MAX
} escProtocol_t;

static const escProtocol_t ESC_PROTOCOL = ESC_DSHOT600;
