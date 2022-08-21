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

#include "arming.h"
#include "datatypes.h"

class Imu {

    public:

        virtual void accumulateGyro(float gx, float gy, float gz)
        {
            (void)gx;
            (void)gy;
            (void)gz;
        }

        virtual void getEulerAngles(
                imu_fusion_t * fusionPrev,
                Arming::data_t * arming,
                uint32_t time,
                vehicle_state_t * vstate) = 0;
};

#if defined(__cplusplus)
extern "C" {
#endif

    void  imuDevInit(uint8_t interruptPin);

#if defined(__cplusplus)
}
#endif
