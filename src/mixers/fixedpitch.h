#pragma once

/*
   Common mixer function for fixed-pitch vehicles

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

#include "datatypes.h"

static void fixedPitchMix(
        float roll,
        float pitch,
        float yaw,
        axes_t axes[],
        uint8_t motorCount, 
        float motors[])
{
    for (int i=0; i<motorCount; i++) {
        float mix = roll * axes[i].x + pitch * axes[i].y + yaw * axes[i].z;
        motors[i] = mix;
    }
}
