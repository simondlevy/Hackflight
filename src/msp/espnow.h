/*
   Copyright (c) 2022 Simon D. Levy

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

#pragma once

#include <esp_now.h>
#include <WiFi.h>

#include "msp.h"
#include "debugger.h"

class EspNowMsp : public Msp {

    private:

        // Replace with the MAC Address of your ESPNOW receiver ---------------
        const uint8_t RECEIVER_ADDRESS[6] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6C, 0x04};

    public:

        void begin(void)
        {
            WiFi.mode(WIFI_STA);

            if (esp_now_init() != ESP_OK) {
                HfDebugger::reportForever("Error initializing ESP-NOW");
            }

            static esp_now_peer_info_t peerInfo;

            memcpy(peerInfo.peer_addr, RECEIVER_ADDRESS, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                HfDebugger::reportForever("Failed to add peer");
            }
        }

        void sendPayload(void) override
        {
            esp_now_send(RECEIVER_ADDRESS, m_payload, m_payloadSize);
        }

}; // class ArduinoMsp
