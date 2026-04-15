/*
 * Hackflight ESPNOW radio dongle sketch
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

// Flying
static const uint8_t RECEIVER_ADDRESS[] = {0x8C,0xBF,0xEA,0xCB,0x8F,0x94};

// Proto-board
//static const uint8_t RECEIVER_ADDRESS[] = {0x54,0x32,0x04,0x33,0x0D,0xF0};


void serialEvent()
{
    const auto avail = Serial.available();

    uint8_t buf[256] = {};

    Serial.read(buf, avail);

    esp_now_send(RECEIVER_ADDRESS, buf, avail);
}

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        reportForever("Error initializing ESP-NOW");
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        reportForever("Failed to add peer");
    }
}

void loop()
{
}
