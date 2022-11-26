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


//  Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <hackflight.h>
#include <msp/parser.h>
#include <espnow.h>

#include <esp_now.h>

static const bool UART_INVERTED = true;

static const uint8_t RX_PIN = 4; // unused
static const uint8_t TX_PIN = 14;

static MspSerializer _serializer;

void setup()
{
    Serial.begin(115200);

    // Start outgoing serial connection to FC, inverted
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN, UART_INVERTED);
}

void loop()
{
    delay(1);
}
