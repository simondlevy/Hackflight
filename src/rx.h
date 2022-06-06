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

#include <stdint.h>

#include "pids/angle_struct.h"
#include "time.h"

#if defined(__cplusplus)
extern "C" {
#endif

    // For both hardware and sim implementations -----------------------------------------

    void rxDevInit(void);

    void rxGetDemands(timeUs_t currentTimeUs, angle_pid_t * ratepid, demands_t * demands);

    void rxPoll(
            timeUs_t currentTimeUs,
            bool imuIsLevel,
            bool calibrating,
            rx_axes_t * rxax,
            bool * pidItermResetReady,
            bool * pidItermResetValue,
            bool * armed,
            bool * gotNewData);

#if defined(__cplusplus)
}
#endif

// For hardware impelmentations ------------------------------------------------------

bool rxCheck(timeUs_t currentTimeUs);

uint8_t rxDevCheck(uint16_t * channelData, timeUs_t * frameTimeUs);

float rxDevConvertValue(uint16_t * channelData, uint8_t chan);

void rxProcessDataModes(
        timeUs_t currentTimeUs,
        bool signalReceived,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool * armed);

void rxUpdateArmingStatus(
        timeUs_t currentTimeUs,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool armed);



