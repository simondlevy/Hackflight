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

#include <Wire.h>

#include <hackflight.hpp>
#include <espnow_utils.hpp>
#include <msp.hpp>
#include <i2c_comms.h>

static uint8_t TX_ADDRESS[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4};
static uint8_t DONGLE_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};

static uint8_t RX_PIN = 6; // unused
static uint8_t TX_PIN = 7;

static const uint32_t UPDATE_FREQ = 50;

// Callback for data received from transmitter
static void OnDataRecv(const uint8_t * mac, const uint8_t * data, int len) 
{
    Serial1.write(data, len);
}

void setup() 
{
    // Act as an I^2C host device for Teensy
    Wire.begin();

    // Enable serial debugging
    Serial.begin(115200);

    // Start ESP-NOW
    hf::EspNowUtils::init();

    // Add the telemetry dongle as a perr
    hf::EspNowUtils::addPeer(DONGLE_ADDRESS);

    // Add the transmitter as a peer
    hf::EspNowUtils::addPeer(TX_ADDRESS);

    // Set up to receive messages from the transmitter
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // Set up serial connection for sending RX messages to Teensy
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() 
{
    // Request state-message bytes from Teensy
    const auto bytesReceived = Wire.requestFrom(
            hf::I2C_DEV_ADDR, 
            hf::Msp::STATE_MESSAGE_SIZE);

    // Once state-message bytes are avaialble
    if (bytesReceived == hf::Msp::STATE_MESSAGE_SIZE) {

        // Read the message bytes from the Teensy
        uint8_t msg[bytesReceived] = {};
        Wire.readBytes(msg, bytesReceived);

        // Send the message bytes to the dongle
        const auto result = esp_now_send(DONGLE_ADDRESS, msg, hf::Msp::STATE_MESSAGE_SIZE);
        if (result != ESP_OK) {
            Serial.printf("Error sending the data\n");
        }
    }

    // Wait a little to avoid overwhelming the dongle with messages
    delay(1000/UPDATE_FREQ);
}

