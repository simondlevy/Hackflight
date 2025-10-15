/**
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <bootloader.hpp>

static const uint8_t RX_PIN = 5;
static const uint8_t TX_PIN = 6;

static HardwareSerial serial = HardwareSerial(RX_PIN, TX_PIN);

void serialEvent()
{
    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}


void setup()
{
    Serial.begin(115200);

    serial.begin(115200);
}


void loop()
{  
    static uint8_t k;

    serial.write((char)('A' + k));

    k = (k + 1) % 26;

    delay(50);
}
