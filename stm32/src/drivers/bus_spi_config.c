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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io.h"
#include "resource.h"
#include "systemdev.h"

#include "flash.h"

typedef struct spiPreinit_s {
    ioTag_t iotag;
    uint8_t iocfg;
    bool init;
} spiPreinit_t;

#define SPI_PREINIT_COUNT 16

static spiPreinit_t spiPreinitArray[SPI_PREINIT_COUNT];
static int spiPreinitCount = 0;

void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, bool init)
{
    if (!iotag) {
        return;
    }

    if (spiPreinitCount == SPI_PREINIT_COUNT) {
        systemIndicateFailure(FAILURE_DEVELOPER, 5);
        return;
    }

    spiPreinitArray[spiPreinitCount].iotag = iotag;
    spiPreinitArray[spiPreinitCount].iocfg = iocfg;
    spiPreinitArray[spiPreinitCount].init = init;
    ++spiPreinitCount;
}

static void spiPreinitPin(spiPreinit_t *preinit, int index)
{
    IO_t io = IOGetByTag(preinit->iotag);
    IOInit(io, OWNER_PREINIT, RESOURCE_INDEX(index));
    IOConfigGPIO(io, preinit->iocfg);
    if (preinit->init) {
        IOHi(io);
    } else {
        IOLo(io);
    }
}

void spiPreInit(void)
{
    flashPreInit();

    for (int i = 0; i < spiPreinitCount; i++) {
        spiPreinitPin(&spiPreinitArray[i], i);
    }
}

void spiPreinitByIO(IO_t io)
{
    for (int i = 0; i < spiPreinitCount; i++) {
        if (io == IOGetByTag(spiPreinitArray[i].iotag)) {
            spiPreinitPin(&spiPreinitArray[i], i);
            return;
        }
    }
}

void spiPreinitByTag(ioTag_t tag)
{
    spiPreinitByIO(IOGetByTag(tag));
}
