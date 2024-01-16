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

static const uint8_t RX1_PIN = 2; // unused;
static const uint8_t TX1_PIN = 4; 

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len) 
{
    Serial.printf("got %d bytes\n", len);
    Serial1.write(incomingData, len);
}

static void error(const char * message)
{
    while (true) {
        Serial.println(message);
        delay(500);
    }
}

void setup() 
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Begin serial connection to flight controller
    Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        error("Error initializing ESP-NOW");
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
}
