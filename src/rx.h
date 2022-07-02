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

#if defined(__cplusplus)
extern "C" {
#endif

    // For both hardware and sim implementations -------------------------------

    void rxDevInit(serialPortIdentifier_e port);

    void rxGetDemands(
            rx_t * rx,
            uint32_t currentTimeUs,
            angle_pid_t * ratepid,
            demands_t * demands);

    void rxPoll(
            rx_t * rx,
            uint32_t currentTimeUs,
            bool imuIsLevel,
            bool calibrating,
            rx_axes_t * rxax,
            void * motorDevice,
            arming_t * arming,
            bool * pidItermResetReady,
            bool * pidItermResetValue,
            bool * gotNewData);

#if defined(__cplusplus)
}
#endif

// For hardware impelmentations ------------------------------------------------

bool rxCheck(rx_t * rx, uint32_t currentTimeUs);

uint8_t rxDevCheck(uint16_t * channelData, uint32_t * frameTimeUs);

float rxDevConvert(uint16_t * channelData, uint8_t chan);
