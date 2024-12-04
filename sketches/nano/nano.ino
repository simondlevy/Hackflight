/*
   ESP32 TinyPICO Nano sketch

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
#include <esp32.hpp>
#include <msp.hpp>
#include <i2c_comms.h>

//static uint8_t DONGLE_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};
static uint8_t DONGLE_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0x97, 0x90};

// Callback for data received from ESP32 dongle
static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    static hf::Msp _msp;

    for (int k=0; k<len; ++k) {

        Serial1.write(incomingData[k]);
    }
}

void setup() 
{
    // Act as an I^2C host device
    Wire.begin();

    Serial.begin(115200);

    Esp32Now::init(DONGLE_ADDRESS, OnDataRecv);

    Serial1.begin(115200, SERIAL_8N1, 4, 14);
}

void loop() 
{
    static bool _lost_wifi;

    const auto bytesReceived = Wire.requestFrom(
            hf::I2C_DEV_ADDR, 
            hf::Msp::STATE_MESSAGE_SIZE);

    if (bytesReceived == hf::Msp::STATE_MESSAGE_SIZE) {

        uint8_t msg[bytesReceived] = {};

        Wire.readBytes(msg, bytesReceived);

        const auto result = esp_now_send(DONGLE_ADDRESS, msg, hf::Msp::STATE_MESSAGE_SIZE);

        if (result != ESP_OK) {
            Serial.printf("Error sending the data\n");
        }
        else {
            static uint32_t _count;
            Serial.printf("%ld\n", _count++);
        }
    }

    delay(10);
}

