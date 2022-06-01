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

#include <Arduino.h>

#include <stdbool.h>
#include <stdint.h>

#include "time.h"

extern "C" {

void motorStop(void)
{
}

float motorConvertFromExternal(uint16_t externalValue)
{
    return 0;
}

bool motorIsProtocolDshot(void)
{
    return false;
}

bool motorDshotStreamingCommandsAreEnabled(void)
{
    return false;
}

void motorCheckDshotBitbangStatus(void)
{
}

bool motorCheckDshotReady(timeUs_t currentTime, uint8_t * tryingToArm)
{
    return false;
}

uint16_t motorGetDshotTelemetry(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

bool motorIsProtocolEnabled(void)
{
    return false;
}

float motorValueDisarmed(void)
{
    return 0;
}

float motorValueHigh(void)
{
    return 0;
}

float motorValueLow(void)
{
    return 0;
}

void motorWrite(float *values)
{
    (void)values;
}

} // extern "C"
