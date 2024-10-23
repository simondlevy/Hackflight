/*
   ESP32 dongl3 sketch

   Relays SET_RC messages from GCS program to onboard ESP32; receives telemetry
   from onboard ESP32 and relays to GCS.

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

#include <hackflight.hpp>
#include <esp32.hpp>

static uint8_t ONBOARD_ADDRESS[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4};

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    Serial.write(incomingData, len);
}

void setup() 
{
    Serial.begin(115200);

    Esp32Now::init(ONBOARD_ADDRESS, OnDataRecv);
}

void loop() 
{
    uint8_t bytes[18] = {};

    Serial.readBytes(bytes, 18);

    esp_err_t result = esp_now_send(ONBOARD_ADDRESS, bytes, 18);

    if (result != ESP_OK) {
        // Serial.println("Error sending the data");
    }

    delay(10);
}

