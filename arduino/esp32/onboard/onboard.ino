/*
   ESP32 on-board sketch

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

// Replace with the MAC Address of your ESPNOW receiver
static const uint8_t ESP_RECEIVER_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void startEspNow(void)
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        reportForever("Error initializing ESP-NOW");
    }

    static esp_now_peer_info_t peerInfo;

    memcpy(peerInfo.peer_addr, ESP_RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        reportForever("Failed to add peer");
    }
}

void setup()
{
    Serial.begin(115200);

    startEspNow();
}

void loop()
{
    const uint8_t s[1] = {99};

    esp_now_send(ESP_RECEIVER_ADDRESS, s, 1);
}

