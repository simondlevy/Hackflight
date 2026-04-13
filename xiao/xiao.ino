/*
 * Hackflight ESPNOW onboard radio sketch
 *
 * Copyright (C) 2026 Simon D. Levy
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

#include <esp_now.h>
#include <WiFi.h>

static HardwareSerial serial(1);

/*
static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (auto k=0; k<len; ++k) {

        serial.write(incomingData[k]);
    }
}*/

void setup(void)
{
    serial.begin(115200, SERIAL_8N1, 0, 1); 

    /*
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {

        while (true) {
            Serial.println("Error initializing ESP-NOW");
            delay(500);
        }
    }

    esp_now_register_recv_cb(onDataRecv);*/
}


void loop(void)
{
    static uint8_t _k;

    serial.write('A' + _k);

    _k = (_k + 1) % 26;
}
