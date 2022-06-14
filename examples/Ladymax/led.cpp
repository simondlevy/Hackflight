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

#include <led.h>

static const uint8_t PIN = A4;

static bool _on;

void ledInit(void)
{
    pinMode(PIN, OUTPUT);
}

void ledSet(bool on)
{
    digitalWrite(PIN, on);
    _on = on;
}

void ledToggle(void)
{
    _on = !_on;
    ledSet(_on);
}
