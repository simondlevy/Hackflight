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

#include <espnow.h>

static const uint8_t LED_PIN         = 25;

static void updateLed(void)
{
    static uint32_t _prev;
    static bool _state;

    uint32_t msec = millis();

    if (msec - _prev > 500) {
        _state = !_state;
        digitalWrite(LED_PIN, _state);
        _prev = msec;
    }
}

// Replace with the MAC Address of your receiver 
static EspNow _esp = EspNow(0xAC, 0x0B, 0xFB, 0x6F, 0x6C, 0x04);

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    _esp.begin();
}

void loop()
{
    updateLed();

    static uint8_t count;
    _esp.send(&count, 1);
    count = (count+1) % 256;
}
