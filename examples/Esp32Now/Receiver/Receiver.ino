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
#include <task/msp/esp32.h>

#include <esp_now.h>

// Replace with the MAC Address of your sender 
static Esp32Msp _msp = Esp32Msp(0xAC, 0x0B, 0xFB, 0x6F, 0x6E, 0x84);

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (uint8_t k=0; k<len; ++k) {
        _msp.parse(incomingData[k]);
    }

    delay(1);
}

void setup()
{
    Serial.begin(115200);

    // Start ESP32 MSP
    _msp.begin();

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(onDataRecv);
}

void loop()
{
}
