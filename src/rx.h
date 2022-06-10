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

#pragma once

#include <math.h>

#include "datatypes.h"
#include "serial.h"
#include "time.h"

static const uint8_t RC_EXPO = 0;
static const uint8_t RC_RATE = 7;
static const uint8_t RATE    = 67;

#if defined(__cplusplus)
extern "C" {
#endif

    // For both hardware and sim implementations -----------------------------------------

    void rxDevInit(serialPortIdentifier_e port);

    void rxGetDemands(uint32_t currentTimeUs, angle_pid_t * ratepid, demands_t * demands);

    void rxPoll(
            uint32_t currentTimeUs,
            bool imuIsLevel,
            bool calibrating,
            rx_axes_t * rxax,
            bool * pidItermResetReady,
            bool * pidItermResetValue,
            bool * armed,
            bool * gotNewData);

static float rxApplyRates(float commandf, const float commandfAbs)
{
    float expof = RC_EXPO / 100.0f;
    expof = commandfAbs * (powf(commandf, 5) * expof + commandf * (1 - expof));

    const float centerSensitivity = RC_RATE * 10.0f;
    const float stickMovement = fmaxf(0, RATE * 10.0f - centerSensitivity);
    const float angleRate = commandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

static throttleStatus_e rxCalculateThrottleStatus(float * rcData)
{
    return (rcData[THROTTLE] < 1050) ?  THROTTLE_LOW : THROTTLE_HIGH;
}
#if defined(__cplusplus)
}
#endif

// For hardware impelmentations ------------------------------------------------------

bool rxCheck(uint32_t currentTimeUs);

uint8_t rxDevCheck(uint16_t * channelData, uint32_t * frameTimeUs);

float rxDevConvertValue(uint16_t * channelData, uint8_t chan);

void rxProcessDataModes(
        uint32_t currentTimeUs,
        bool signalReceived,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool * armed);

void rxUpdateArmingStatus(
        uint32_t currentTimeUs,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool armed);



