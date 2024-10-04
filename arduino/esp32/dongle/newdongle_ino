/*
 * Listens for messages from Skryanger over ESP-NOW

   Copyright (c) 2023 Simon D. Levy

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

#include <esp_now.h>
#include <WiFi.h>

#include "msp.h"

// Message IDs
static const uint8_t MSP_SET_VL53L5   = 221;
static const uint8_t MSP_SET_PAA3905  = 222;

static Msp _parser;

static uint32_t _vl53l5_count;
static uint32_t _paa3905_count;

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (auto k=0; k<len; ++k) {

        switch (_parser.parse(incomingData[k])) {
            
            case MSP_SET_VL53L5:
                _vl53l5_count++;
                break;

            case MSP_SET_PAA3905:
                _paa3905_count++;
                break;
        }
    }
}

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void setup(void)
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {

        reportForever("Error initializing ESP-NOW");
    }

    esp_now_register_recv_cb(onDataRecv);
}


void loop(void)
{
    printf("Got %d VL53L5 messages and %d PAA3905 messages\n",
            _vl53l5_count, _paa3905_count);
}
