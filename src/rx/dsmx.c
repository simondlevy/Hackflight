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

// Support DSMX2048 only
static const uint8_t CHAN_SHIFT = 3;
static const uint8_t CHAN_MASK  = 0x07;
static const uint8_t MAX_CHANNELS = 8;

static const uint16_t FRAME_INTERVAL = 5000;

static volatile uint8_t _frame[FRAME_SIZE];
static bool _frameComplete;
static uint32_t _lastFrameTimeUs;

// Receive ISR callback
static void dsmxDataReceive(uint8_t c, void *data, uint32_t currentTimeUs)
{
    (void)data;

    static uint32_t _spekTimeLast;
    static uint8_t _framePosition;

    const uint32_t spekTimeInterval = cmpTimeUs(currentTimeUs, _spekTimeLast);

    _spekTimeLast = currentTimeUs;

    if (spekTimeInterval > FRAME_INTERVAL) {
        _framePosition = 0;
    }

    if (_framePosition < FRAME_SIZE) {
        _frame[_framePosition++] = (uint8_t)c;
        if (_framePosition < FRAME_SIZE) {
            _frameComplete = false;
        } else {
            _lastFrameTimeUs = currentTimeUs;
            _frameComplete = true;
        }
    }
}

// Public API ==================================================================

uint8_t rxDevCheck(uint16_t * channelData, uint32_t * frameTimeUs)
{
    uint8_t result = RX_FRAME_PENDING;

    if (_frameComplete) {

        _frameComplete = false;

        *frameTimeUs = _lastFrameTimeUs;

        for (int b = 3; b < FRAME_SIZE; b += 2) {

            const uint8_t channel = 0x0F & (_frame[b - 1] >> CHAN_SHIFT);

            if (channel < MAX_CHANNELS) {

                channelData[channel] =
                    ((uint32_t)(_frame[b - 1] & CHAN_MASK) << 8) + _frame[b];
            }
        }

        result = RX_FRAME_COMPLETE;
    }

    return result;
}

float rxDevConvertValue(uint16_t * channelData, uint8_t chan)
{
    (void)chan;

    printf("%d\n", channelData[0]);

    return 0;
}

void rxDevInit(serialPortIdentifier_e port)
{
    serialOpenPortDsmx(port, dsmxDataReceive);
}
