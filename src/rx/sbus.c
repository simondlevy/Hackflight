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

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
 *
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

static const uint32_t FLAG_SIGNAL_LOSS     = (1 << 2);
static const uint32_t FLAG_FAILSAFE_ACTIVE = (1 << 3);
static const uint32_t FLAG_CHANNEL_17      = (1 << 0);
static const uint32_t FLAG_CHANNEL_18      = (1 << 1);

static const uint16_t DIGITAL_CHANNEL_MIN = 173;
static const uint16_t DIGITAL_CHANNEL_MAX = 1812;

static const uint32_t TIME_NEEDED_PER_FRAME = 3000;

static const uint8_t FRAME_BEGIN_BYTE = 0x0F;

enum {
    DEBUG_FRAME_FLAGS = 0,
    DEBUG_STATE_FLAGS,
    DEBUG_FRAME_TIME,
};

typedef struct sbusChannels_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    uint16_t chan0 : 11;
    uint16_t chan1 : 11;
    uint16_t chan2 : 11;
    uint16_t chan3 : 11;
    uint16_t chan4 : 11;
    uint16_t chan5 : 11;
    uint16_t chan6 : 11;
    uint16_t chan7 : 11;
    uint16_t chan8 : 11;
    uint16_t chan9 : 11;
    uint16_t chan10 : 11;
    uint16_t chan11 : 11;
    uint16_t chan12 : 11;
    uint16_t chan13 : 11;
    uint16_t chan14 : 11;
    uint16_t chan15 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbusChannels_t;

#define FRAME_SIZE (sizeof(sbusChannels_t) + 2)

struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleancleanissues/590#issuecomment-101027349
     * and
     * https://github.com/cleancleanissues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbusFrame_u {
    uint8_t bytes[FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s {
    sbusFrame_t frame;
    uint32_t startAtUs;
    uint8_t position;
    bool done;
} sbusFrameData_t;

static uint8_t sbusChannelsDecode(uint16_t * channelData, const sbusChannels_t *channels)
{
    channelData[0] = channels->chan0;
    channelData[1] = channels->chan1;
    channelData[2] = channels->chan2;
    channelData[3] = channels->chan3;
    channelData[4] = channels->chan4;
    channelData[5] = channels->chan5;
    channelData[6] = channels->chan6;
    channelData[7] = channels->chan7;
    channelData[8] = channels->chan8;
    channelData[9] = channels->chan9;
    channelData[10] = channels->chan10;
    channelData[11] = channels->chan11;
    channelData[12] = channels->chan12;
    channelData[13] = channels->chan13;
    channelData[14] = channels->chan14;
    channelData[15] = channels->chan15;

    if (channels->flags & FLAG_CHANNEL_17) {
        channelData[16] = DIGITAL_CHANNEL_MAX;
    } else {
        channelData[16] = DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & FLAG_CHANNEL_18) {
        channelData[17] = DIGITAL_CHANNEL_MAX;
    } else {
        channelData[17] = DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & FLAG_FAILSAFE_ACTIVE) {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (channels->flags & FLAG_SIGNAL_LOSS) {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

static sbusFrameData_t _frameData;

// Receive ISR callback
static void sbusDataReceive(uint8_t c, void *data, uint32_t currentTimeUs)
{
    (void)data;

    const uint32_t nowUs = currentTimeUs;

    const int32_t sbusFrameTime = cmpTimeUs(nowUs, _frameData.startAtUs);

    if (sbusFrameTime > (long)(TIME_NEEDED_PER_FRAME + 500)) {
        _frameData.position = 0;
    }

    if (_frameData.position == 0) {
        if (c != FRAME_BEGIN_BYTE) {
            return;
        }
        _frameData.startAtUs = nowUs;
    }

    if (_frameData.position < FRAME_SIZE) {
        _frameData.frame.bytes[_frameData.position++] = (uint8_t)c;
        if (_frameData.position < FRAME_SIZE) {
            _frameData.done = false;
        } else {
            _frameData.done = true;
        }
    }
}

// Public API ==================================================================

uint8_t rxDevCheck(uint16_t * channelData, uint32_t * frameTimeUs)
{
    if (!_frameData.done) {
        return RX_FRAME_PENDING;
    }
    _frameData.done = false;

    const uint8_t frameStatus = sbusChannelsDecode(channelData,
            &_frameData.frame.frame.channels);

    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        *frameTimeUs = _frameData.startAtUs;
    }

    return frameStatus;
}

float rxDevConvertValue(uint16_t * channelData, uint8_t chan)
{
    return (5 * (float)channelData[chan] / 8) + 880;
}

void rxDevInit(serialPortIdentifier_e port)
{
    serialOpenPortSbus(port, sbusDataReceive);
}
