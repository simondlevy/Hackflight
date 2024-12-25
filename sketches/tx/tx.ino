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
#include <espnow_helper.hpp>

// MSP support
#include <msp.hpp>

// Address of TinyPICO flight controller
static uint8_t FC_ADDRESS[] = {0xD4, 0xD4, 0xDA, 0x84, 0xDC, 0xB4};

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
    hf::EspNowHelper::init();

    // Add receiver as peer
    hf::EspNowHelper::addPeer(FC_ADDRESS);
}

void loop()
{
    if (_sbus.Read()) {

        const auto data = _sbus.data();

        /*
        Serial.printf("c1=%04d  c2=%04d  c3=%04d  c4=%04d  c5=%04d  c6=%04d\n",
                data.ch[0], data.ch[1], data.ch[2],
                data.ch[3], data.ch[4], data.ch[5]);*/

        // Create an MSP message from the channel values
        _msp.serializeShorts(200, data.ch, 6);

        // Send the message bytes to the receiver
        hf::EspNowHelper::sendToPeer(
                FC_ADDRESS, _msp.payload, _msp.payloadSize, "tx", "fc");
    }
}
