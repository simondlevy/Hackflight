/*
   ESP32 dongle sketch

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

#include <hackflight.hpp>
#include <espnow/utils.hpp>
#include <msp.hpp>

static constexpr uint8_t TELEMETRY_DONGLE_ADDRESS[6] = {
    0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4
};

static constexpr uint8_t TRANSMITTER_ADDRESS[6] = {
    0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4
};

// Handles EPS-NOW SET_RC messages from transmitter, sending them to Teensy
// over UART.
void espnowEvent(const uint8_t * mac, const uint8_t * data, int len) 
{
    (void)mac;

    Serial1.write(data, len);
}

// Handles streaming STATE telemetry messages from Teensy:  collects message
// bytes, and when a complete message is received, sends all the message bytes
// to Teensy.
void serialEvent1()
{
    static hf::Msp _msp;

    static uint8_t _msg[256];
    static uint8_t _msgcount;

    while (Serial1.available()) {

        const auto c = Serial1.read();

        _msg[_msgcount++] = c;

        if (_msp.parse(c)) {

            hf::EspNowUtils::sendToPeer(
                    TELEMETRY_DONGLE_ADDRESS, _msg, _msgcount, "nano", "dongle");

            _msgcount = 0;
        }
    }
}

void setup() 
{
    // Set up for serial debugging
    Serial.begin(115200);

    // Set up for receiving serial telemetry data from Teensy
    Serial1.begin(115200, SERIAL_8N1, 4, 14 );

    // Start ESP-NOW
    hf::EspNowUtils::init();

    // Add the telemetry dongle as a peer
    hf::EspNowUtils::addPeer(TELEMETRY_DONGLE_ADDRESS);

    // Add the transmitter as a peer
    hf::EspNowUtils::addPeer(TRANSMITTER_ADDRESS);

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(espnowEvent));
}

void loop() 
{
}
