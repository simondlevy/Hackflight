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

// Constants ----------------------------------------------------------

// Replace with the MAC Address of your ESPNOW receiver
static const uint8_t ESP_RECEIVER_ADDRESS[] = {0xD8, 0xA0, 0x1D, 0x54, 0x87, 0x44};

// Pins
static const uint8_t FC_RX_PIN = 4;
static const uint8_t FC_TX_PIN = 14;


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

// Attitude messages from FC ----------------------------------------

void serialEvent1(void)
{
    while (Serial1.available()) {

        uint8_t byte = Serial1.read();

        // Send message bytes directly to ESP receiver
        esp_now_send(ESP_RECEIVER_ADDRESS, &byte, 1);
    }
}

// ------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    // startEspNow();
}

void loop()
{
    static uint8_t k;

    Serial1.write('A' + k);

    k = (k+1) % 26;

    delay(10);
}
