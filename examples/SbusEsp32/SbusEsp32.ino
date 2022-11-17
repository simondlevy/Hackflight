/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <tasks/receivers/sbus.h>

static const uint8_t RX_PIN = 4;
static const uint8_t TX_PIN = 14; // unused

static SbusReceiver _rx;


void setup(void)
{
    Serial.begin(115200);

    Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true);
}

void loop(void)
{
    while (Serial1.available()) {
        _rx.parse(Serial1.read());
    }

    Serial.println(_rx.ready());

    delay(5);
}
