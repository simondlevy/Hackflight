/*
   Copyright (c) 2024 Simon D. Levy

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

//   Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

// Bolderflight's SBUS library
#include <sbus.h>

// ESP-NOW support
#include <espnow_utils.hpp>

// MSP support
#include <msp.hpp>

// Address of TinyPICO Nano receiver 
static uint8_t RX_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x83, 0xD4, 0x40};

// Support for SBUS from FrSky transmitter
static bfs::SbusRx _sbus = bfs::SbusRx(&Serial1, 25, 26, true);

// Support for serializing messages
static hf::Msp _msp;

void setup()
{
    // Set up serial debugging
    Serial.begin(115200);

    // Start incoming SBUS connection from TX
    _sbus.Begin();

    // Start ESP now comms
    hf::EspNowUtils::init();

    // Add receiver as peer
    hf::EspNowUtils::addPeer(RX_ADDRESS);
}

void loop()
{
    if (_sbus.Read()) {

        const auto data = _sbus.data();

        Serial.printf("c1=%04d  c2=%04d  c3=%04d  c4=%04d  c5=%04d  c6=%04d\n",
                data.ch[0], data.ch[1], data.ch[2],
                data.ch[3], data.ch[4], data.ch[5]);

        // Create an MSP message from the channel values
        _msp.serializeShorts(200, data.ch, 6);

        // Send the message bytes to the receiver
        const auto result = esp_now_send(RX_ADDRESS, _msp.payload, 12);
        if (result != ESP_OK) {
            Serial.printf("txadapter: error sending the data\n");
        }

    }
}
