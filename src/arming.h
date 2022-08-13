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
#include <stdbool.h>

#include "datatypes.h"

    void armingCheck(
            arming_t * arming,
            void * motorDevice,
            uint32_t currentTimeUs,
            float raw[],
            bool imuIsLevel,
            bool calibrating);

    void  armingDisarm(arming_t * arming, void * motorDevice);

    bool  armingIsArmed(arming_t * arming);

    void  armingUpdateStatus(
            arming_t * arming,
            float raw[],
            bool imuIsLevel,
            bool calibrating);


    void armingSetRxFailsafe(arming_t * arming, bool enabled);
