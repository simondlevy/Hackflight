/*
   Copyright (c) 2023 Simon D. Levy

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

#include <esp_now.h>
#include <WiFi.h>

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (auto k=0; k<len; ++k) {
        Serial.println(incomingData[k]);
    }
}

void setup(void)
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        while (true) {
            Serial.println("Error initializing ESP-NOW");
            delay(500);
        }
    }

    esp_now_register_recv_cb(onDataRecv);
}


void loop(void)
{
}
