/*
   ESP32 on-board sketch

   Relays serial comms data from Teensy flight-control board to a remote ESP32
   dongle; accepts SET_RC messages from dongle to set RC channel values.

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
#include <Wire.h>

#include <hackflight.hpp>
#include <esp32now_helper.hpp>
#include <msp.hpp>
#include <tasks/comms.hpp>

static uint8_t DONGLE_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    static Msp _msp;

    for (int k=0; k<len; ++k) {

        const auto msgtype = _msp.parse(incomingData[k]);

        if (msgtype == 200) { // SET_RC message

            for (uint8_t k=0; k<6; ++k) {
                Serial.printf("%04d ", _msp.parseShort(k));
            }

            Serial.printf("\n");
        }
    }
}

void setup() 
{
    // Act as an I^2C host device
    Wire.begin();

    Serial.begin(115200);

    Esp32Now::init(DONGLE_ADDRESS, OnDataRecv);
}

void loop() 
{
    const auto bytesReceived = Wire.requestFrom(
            hf::CommsTask::I2C_DEV_ADDR, 
            hf::MSP_STATE_MESSAGE_SIZE);

    if (bytesReceived == hf::MSP_STATE_MESSAGE_SIZE) {

        uint8_t msg[bytesReceived] = {};

        Wire.readBytes(msg, bytesReceived);

        const auto result = esp_now_send(DONGLE_ADDRESS, msg, hf::MSP_STATE_MESSAGE_SIZE);

        if (result != ESP_OK) {
            Serial.printf("Error sending the data\n");
        }
    }

    delay(10);
}

