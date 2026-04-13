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

static void onDataRecv(const uint8_t * mac, const uint8_t *data, int len)
{
    (void)mac;

    serial.write(data, len);

    delay(1);
}

void setup(void)
{
    serial.begin(115200, SERIAL_8N1, 0, 1); 

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {

        while (true) {
            Serial.println("Error initializing ESP-NOW");
            delay(500);
        }
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
}


void loop(void)
{
}
