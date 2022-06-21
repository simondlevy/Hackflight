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

#include "led.h"

static bool _on;
static uint8_t _pin;

void ledInit(uint8_t pin)
{
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void ledSet(bool on)
{
    digitalWrite(_pin, on);
    _on = on;
}

void ledToggle(void)
{
    _on = !_on;
    ledSet(_on);
}
