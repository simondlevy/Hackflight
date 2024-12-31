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

void OnDataRecv(const uint8_t * mac, const uint8_t * data, int len) 
{
    (void)mac;

    for (int k=0; k<len; ++k) {
        Serial.write(data[k]);
    }
}

void setup() 
{
    // We will write binary serial data directly to host program
    Serial.begin(115200);

    // Start ESP-NOW
    hf::EspNowUtils::init();

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() 
{
}
