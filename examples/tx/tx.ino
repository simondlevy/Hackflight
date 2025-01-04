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
#include <hackflight.hpp>
#include <espnow/utils.hpp>

// MSP support
#include <msp/serializer.hpp>

// Address of TinyPICO Nano receiver
static uint8_t RX_ADDRESS[] = {0xE8, 0x6B, 0xEA, 0x24, 0xF6, 0xD0};

// Support for SBUS from FrSky transmitter
static bfs::SbusRx _sbus = bfs::SbusRx(&Serial1, 25, 26, true);

// Support for serializing messages
static hf::MspSerializer _serializer;

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

        // Create an MSP message from the channel values
        _serializer.serializeShorts(200, data.ch, 6);

        // Send the message bytes to the receiver
        hf::EspNowUtils::sendToPeer(RX_ADDRESS,
                _serializer.payload, _serializer.payloadSize, "tx", "fc");
    }
}
