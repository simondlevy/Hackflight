/*
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

#include <escs/dshot_protocol.h>

#include "bitbang.h"

#if defined (__cplusplus)
extern "C" {
#endif

bool bbEnableMotors(void);
void bbPostInit(dshotProtocol_t protocol);
void bbUpdateComplete(uint8_t motorCount);
bool bbUpdateStart(void);
void bbWrite(uint8_t motorIndex, float value);

void dshotBitbangDevInit(const uint8_t pins[], const uint8_t count);

#if defined (__cplusplus)
}
#endif


