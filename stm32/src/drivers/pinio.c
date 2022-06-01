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

#include <stdint.h>

#include "platform.h"

#include "io.h"
#include "pinio.h"

typedef struct pinioConfig_s {
    ioTag_t ioTag[PINIO_COUNT];
    uint8_t config[PINIO_COUNT];
} pinioConfig_t;

typedef struct pinioRuntime_s {
    IO_t io;
    bool inverted;
    bool state;
} pinioRuntime_t;

static pinioRuntime_t pinioRuntime[PINIO_COUNT];

void pinioInit(void)
{
    pinioConfig_t  pinioConfig;

    for (uint8_t k=0; k<PINIO_COUNT; ++k) {
        pinioConfig.ioTag[k] = 0;
        pinioConfig.config[k] = 1;
    }

    for (int i = 0; i < PINIO_COUNT; i++) {
        IO_t io = IOGetByTag(pinioConfig.ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        switch (pinioConfig.config[i] & PINIO_CONFIG_MODE_MASK) {
        case PINIO_CONFIG_MODE_OUT_PP:
            // Initial state after reset is input, pull-up.
            // Avoid momentary off by presetting the output to hi.
            if (pinioConfig.config[i] & PINIO_CONFIG_OUT_INVERTED) {
                IOHi(io);
            }
            IOConfigGPIO(io, IOCFG_OUT_PP);
            break;
        }

        if (pinioConfig.config[i] & PINIO_CONFIG_OUT_INVERTED)
        {
            pinioRuntime[i].inverted = true;
            IOHi(io);
            pinioRuntime[i].state = true;
        } else {
            pinioRuntime[i].inverted = false;
            IOLo(io);
            pinioRuntime[i].state = false;
        }
        pinioRuntime[i].io = io;
    }
}

void pinioSet(int index, bool on)
{
    bool newState = on ^ pinioRuntime[index].inverted;
    if (newState != pinioRuntime[index].state) {
        IOWrite(pinioRuntime[index].io, newState);
        pinioRuntime[index].state = newState;
    }
}
