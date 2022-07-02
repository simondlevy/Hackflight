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

#include <stdlib.h>

#include "datatypes.h"
#include "debug.h"
#include "serial.h"
#include "time.h"

#define FRAME_SIZE 16

static const uint16_t FRAME_INTERVAL = 5000;

static volatile uint8_t _spekFrame[FRAME_SIZE];
static bool _rcFrameComplete;
static uint32_t _lastRcFrameTimeUs;

// Receive ISR callback
static void dsmxDataReceive(uint8_t c, void *data, uint32_t currentTimeUs)
{
    (void)data;

    static uint32_t _spekTimeLast;
    static uint8_t _spekFramePosition;

    const uint32_t spekTimeInterval = cmpTimeUs(currentTimeUs, _spekTimeLast);

    _spekTimeLast = currentTimeUs;

    if (spekTimeInterval > FRAME_INTERVAL) {
        _spekFramePosition = 0;
    }

    if (_spekFramePosition < FRAME_SIZE) {
        _spekFrame[_spekFramePosition++] = (uint8_t)c;
        if (_spekFramePosition < FRAME_SIZE) {
            _rcFrameComplete = false;
        } else {
            _lastRcFrameTimeUs = currentTimeUs;
            _rcFrameComplete = true;
        }
    }
}

// Public API ==================================================================

uint8_t rxDevCheckDsmx(uint16_t * channelData, uint32_t * frameTimeUs)
{
    (void)channelData;
    (void)frameTimeUs;
    return 0;;
}

float rxDevConvertValueDsmx(uint16_t * channelData, uint8_t chan)
{
    (void)channelData;
    (void)chan;
    return 0;
}

void rxDevInitDsmx(serialPortIdentifier_e port)
{
    serialOpenPortDsmx(port, dsmxDataReceive);
}
