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
#include <esc.h>
#include <pwm.h>

bool escDevIsReady(uint32_t currentTime)
{
    (void)currentTime;
    return true;
}

float escDevConvertFromExternal(void * escDevice, uint16_t externalValue)
{
    (void)externalValue;
    (void)escDevice;

    return (externalValue - PWM_MIN) / (float)(PWM_MAX - PWM_MIN);
}

void  escDevInitBrushed(uint8_t * pins)
{
    for (auto k=0; k<4; ++k) {
        analogWriteFrequency(pins[k], 10000);
        analogWrite(pins[k], 0);
    }
}

bool escDevIsProtocolDshot(void)
{
    return false;
}

float escDevValueDisarmed(void)
{
    return 0;
}

float escDevValueHigh(void)
{
    return 1;
}

float escDevValueLow(void)
{
    return 0;
}

void escDevStop(void * escDevice)
{
    (void)escDevice;
}

void escDevWrite(void * escDevice, float *values)
{
    uint8_t * pins = (uint8_t *)escDevice;

    for (auto k=0; k<4; ++k) {
        analogWrite(pins[k], (uint8_t)(values[k] * 255));
    }

    //printf("%f %f %f %f\n", values[0], values[1], values[2], values[3]);
}
