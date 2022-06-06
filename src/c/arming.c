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

#include <strings.h>

#include "arming.h"
#include "motor.h"

static arming_t status;

void armingDisarm(bool armed)
{
    if (armed) {
        motorStop();
    }
}

uint8_t armingGetDisableFlags(void)
{
    return ffs(status.disabledFlags);
}

bool armingIsDisabled(void)
{
    return status.disabledFlags != 0;
}

void armingSetDisabled(uint8_t flag)
{
    status.disabledFlags |= (1 << flag);
}

void armingUnsetDisabled(uint8_t flag)
{
    status.disabledFlags &= ~(1 << flag);
}
