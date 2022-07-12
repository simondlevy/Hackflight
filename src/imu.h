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

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

    void    imuAccumulateGyro(gyro_t * gyro);

    void    imuGetEulerAngles(hackflight_t * hf, uint32_t time);

    int32_t imuGetGyroSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles);

    void    imuInit(hackflight_t * hf, uint8_t interruptPin);

    void imuUpdateFusion(
            hackflight_t * hf,
            uint32_t time,
            quaternion_t * quat,
            rotation_t * rot);

#if defined(__cplusplus)
}
#endif
