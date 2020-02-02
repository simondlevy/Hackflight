/*
   Test DSMX => SBUS translation

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
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

#include <SBUS.h>
#include <DSMRX.h>

static const uint8_t CHANNELS_IN  = 8;
static const uint8_t CHANNELS_OUT = 16;

SBUS sbus = SBUS(Serial1);

DSM2048 rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        rx.handleSerialEvent(Serial2.read(), micros());
    }
}

void setup(void)
{
    sbus.begin();

    // For DSMX in
    Serial2.begin(115000);

    // For debugging
    Serial.begin(115000);
}

void loop(void)
{
    float invals[CHANNELS_IN] = {0};

    rx.getChannelValuesNormalized(invals, CHANNELS_IN);

    static float outvals[CHANNELS_OUT];

    outvals[0] = invals[0]; // Throttle
    outvals[1] = invals[1]; // Roll
    outvals[2] = invals[2]; // Pitch
    outvals[3] = invals[3]; // Yaw

    outvals[5] = invals[6]; // DSXM Aux1 => SBUS Aux2

    sbus.writeCal(outvals);

    delay(10);
}

