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

#include "time.h"

#define MAX_SUPPORTED_MOTORS 8

#if defined(__cplusplus)
extern "C" {
#endif

void     motorCheckDshotBitbangStatus(void);
bool     motorCheckDshotReady(uint32_t currentTime, uint8_t * tryingToArm);
float    motorConvertFromExternal(uint16_t externalValue);
bool     motorDshotStreamingCommandsAreEnabled(void);
uint16_t motorGetDshotTelemetry(uint8_t motorIndex);
int16_t  motorGetDshotTelemetryMotorInvalidPercent(uint8_t motorIndex);
bool     motorIsProtocolDshot(void);
bool     motorIsProtocolEnabled(void);
float    motorValueDisarmed(void);
float    motorValueHigh(void);
float    motorValueLow(void);
void     motorStop(void);
void     motorWrite(float *values);

#if defined(__cplusplus)
}
#endif
