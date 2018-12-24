/*
   LoiterListener.ino : Test-listener for LoiterBoard

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modiflowy
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mspparser.hpp"

void setup(void)
{
    Serial.begin(115200);
    Serial1.begin(115200);
}

void loop(void)
{
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        Serial.println(c, HEX);
    }
}
