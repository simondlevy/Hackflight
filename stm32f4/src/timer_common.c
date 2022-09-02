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

#include "platform.h"

#include "dma_reqmap.h"
#include "dshot_bitbang.h"
#include "io.h"
#include "timer.h"

#define MAX_TIMER_PINMAP_COUNT 21 // Largest known for F405RG (OMNINXT)

typedef struct timerIOConfig_s {
    ioTag_t ioTag;
    uint8_t index;
    int8_t dmaopt;
} timerIOConfig_t;

const resourceOwner_t freeOwner = { .owner = OWNER_FREE, .resourceIndex = 0 };

static resourceOwner_t timerOwners[MAX_TIMER_PINMAP_COUNT];

static timerIOConfig_t timerIOConfig[MAX_TIMER_PINMAP_COUNT];

const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex)
{

    if (!ioTag || !timerIndex) {
        return NULL;
    }

    uint8_t index = 1;
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            if (index == timerIndex) {
                return &TIMER_HARDWARE[i];
            }
            ++index;
        }
    }

    return NULL;
}

const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag)
{
    uint8_t timerIndex = 0;
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig[i].ioTag == ioTag) {
            timerIndex = timerIOConfig[i].index;

            break;
        }
    }

    return timerGetByTagAndIndex(ioTag, timerIndex);
}

const timerHardware_t *timerGetAllocatedByNumberAndChannel(
        int8_t timerNumber,
        uint16_t timerChannel)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *timer =
            timerGetByTagAndIndex(timerIOConfig[i].ioTag,
                    timerIOConfig[i].index);
        if (timer && timerGetTIMNumber(timer->tim) ==
                timerNumber && timer->channel == timerChannel && timerOwners[i].owner) {
            return timer;
        }
    }

    return dshotBitbangTimerGetAllocatedByNumberAndChannel(timerNumber, timerChannel);
}

const resourceOwner_t *timerGetOwner(const timerHardware_t *timer)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *assignedTimer =
            timerGetByTagAndIndex(timerIOConfig[i].ioTag, timerIOConfig[i].index);
        if (assignedTimer && assignedTimer == timer) {
            return &timerOwners[i];
        }
    }

    return &freeOwner;

    //return dshotBitbangTimerGetOwner(timer);
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    if (!ioTag) {
        return NULL;
    }

    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig[i].ioTag == ioTag) {
            const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, timerIOConfig[i].index);

            if (timerGetOwner(timer)->owner) {
                return NULL;
            }

            timerOwners[i].owner = owner;
            timerOwners[i].resourceIndex = resourceIndex;

            return timer;
        }
    }

    return NULL;
}

ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
    UNUSED(usageFlag);
    UNUSED(index);
    return IO_TAG_NONE;
}

void timerIoInit(void)
{
    timerIOConfig[0].ioTag = 40;
    timerIOConfig[0].index = 2;
    timerIOConfig[0].dmaopt = -1; 

    timerIOConfig[1].ioTag = 56;
    timerIOConfig[1].index = 2;
    timerIOConfig[1].dmaopt = -1; 

    timerIOConfig[2].ioTag = 32;
    timerIOConfig[2].index = 2;
    timerIOConfig[2].dmaopt = -1; 

    timerIOConfig[3].ioTag = 33;
    timerIOConfig[3].index = 2;
    timerIOConfig[3].dmaopt = -1; 

    timerIOConfig[4].ioTag = 19;
    timerIOConfig[4].index = 1;
    timerIOConfig[4].dmaopt = 1; 

    timerIOConfig[5].ioTag = 18;
    timerIOConfig[5].index = 1;
    timerIOConfig[5].dmaopt = 0; 

    timerIOConfig[6].ioTag = 38;
    timerIOConfig[6].index = 1;
    timerIOConfig[6].dmaopt = 0; 

    timerIOConfig[7].ioTag = 24;
    timerIOConfig[7].index = 1;
    timerIOConfig[7].dmaopt = -1; 

    timerIOConfig[8].ioTag = 25;
    timerIOConfig[8].index = 1;
    timerIOConfig[8].dmaopt = 0; 

    timerIOConfig[9].ioTag = 26;
    timerIOConfig[9].index = 1;
    timerIOConfig[9].dmaopt = 0; 
}
