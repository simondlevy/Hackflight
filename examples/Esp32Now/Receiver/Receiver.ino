/*
   Copyright (c) 2022 Simon D. Levy

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


//  Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <hackflight.h>
#include <msp/parser.h>
#include <espnow.h>

#include <esp_now.h>

static const uint8_t RX_PIN = 4; // unused
static const uint8_t TX_PIN = 14;

// Replace with the MAC Address of your sender 
static EspNow _esp = EspNow(0xAC, 0x0B, 0xFB, 0x6F, 0x6E, 0x84);

static MspParser _parser;
static MspSerializer _serializer;

static void dump(int16_t val)
{
    Serial.print(val);
    Serial.print("  ");
}

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (uint8_t k=0; k<len; ++k) {

        if (_parser.parse(incomingData[k]) == 200) {

            uint16_t c1 = _parser.parseShort(0);
            uint16_t c2 = _parser.parseShort(1);
            uint16_t c3 = _parser.parseShort(2);
            uint16_t c4 = _parser.parseShort(3);
            uint16_t c5 = _parser.parseShort(4);
            uint16_t c6 = _parser.parseShort(5);

            /*
            dump(c1);
            dump(c2);
            dump(c3);
            dump(c4);
            dump(c5);
            dump(c6);
            Serial.println();*/

            _serializer.serializeRawRc(200, c1, c2, c3, c4, c5, c6);

            for (uint8_t k=0; k<_serializer.outBufSize; ++k) {
                Serial1.write(_serializer.outBuf[k]);
            }
        }
    }

    delay(1);
}

void setup()
{
    Serial.begin(115200);

    // Start outgoing serial connection to FC, inverted
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN, true);

    // Start ESP32 MSP
    _esp.begin();

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(onDataRecv);
}

void loop()
{
}
