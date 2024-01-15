/*
   Copyright (c) 2024 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

// https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
static const uint8_t RECEIVER_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x84, 0xD5, 0x0C};

static void error(const char * message)
{
    while (true) {
        Serial.println(message);
        delay(500);
    }
}

void setup(void) 
{
    // We will read input from USB port, via serialEvent() callback
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        error("Error initializing ESP-NOW");
    }

    // Register peer
    static esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        error("Failed to add peer");
    }
}

void loop(void) 
{
    const auto avail = Serial.available();

    static uint8_t buf[256];

    Serial.readBytes(buf, avail);

    esp_now_send(RECEIVER_ADDRESS, buf, avail);
}
