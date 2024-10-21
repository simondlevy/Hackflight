/*
   ESP32 dongle sketch

   For now we just read data from onboard ESP32 and send it out over
   USB serial for use by GCS program

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <WiFi.h>

static const uint8_t ESP_ONBOARD_ADDRESS[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4};

// Handles incoming telemetry from onboard ESP32
static void onDataRecv(
        const esp_now_recv_info * info,
        const uint8_t *incomingData,
        int len)
{
    (void)info;

    // Send data to GCS
    Serial.write(incomingData, len);

    //delay(10);
}

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

static void startEspNow(void)
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        reportForever("Error initializing ESP-NOW");
    }

    static esp_now_peer_info_t peerInfo;

    memcpy(peerInfo.peer_addr, ESP_ONBOARD_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        reportForever("Failed to add peer");
    }

    esp_now_register_recv_cb(onDataRecv);
}


void setup(void)
{
    Serial.begin(115200);

    startEspNow();

    /*
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {

        reportForever("Error initializing ESP-NOW");
    }

    esp_now_register_recv_cb(onDataRecv);*/

}


void loop(void)
{
    // Read incoming stick demands from GCS
    while (Serial.available() > 0) {
        const uint8_t b[1] = { Serial.read() };
        esp_now_send(ESP_ONBOARD_ADDRESS, b, 1);
    }

    // A little delay here will minimize read/write contention
    //delay(10);
}
