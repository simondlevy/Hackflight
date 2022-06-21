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

#include <Arduino.h>

#include <arming.h>
#include <motor.h>

void motorCheckDshotBitbangStatus(void)
{
    armingSetDshotBitbang(true);
}

bool motorIsReady(uint32_t currentTime)
{
    (void)currentTime;
    return true;
}

float motorConvertFromExternal(uint16_t externalValue)
{
    (void)externalValue;
    return 0;
}

bool motorIsProtocolDshot(void)
{
    return false;
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
    return 1;
}

float motorValueLow(void)
{
    return 0;
}

void motorStop(void)
{
}

void motorWrite(float *values)
{
}
