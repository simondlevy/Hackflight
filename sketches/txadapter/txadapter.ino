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

static const uint8_t RX_PIN = 25;
static const uint8_t TX_PIN = 26; // unused

static bfs::SbusRx _sbus = bfs::SbusRx(&Serial1, RX_PIN, TX_PIN, true);

/*
#include <hackflight.h>
#include <msp/serializer.h>
#include <task/receiver/sbus.h>
#include <espnow.h>

static SbusReceiver _rx;

static MspSerializer _serializer;

// Replace with the MAC Address of your receiver 
static EspNow _esp = EspNow(0xAC, 0x0B, 0xFB, 0x6F, 0x69, 0xA0);

static uint16_t convert(uint16_t chanval)
{
    return (uint16_t)SbusReceiver::convert(chanval);
}

   void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
   {
   Serial.print("\r\nLast Packet Send Status:\t");

   Serial.println(
   status == ESP_NOW_SEND_SUCCESS ?
   "Delivery Success" :
   "Delivery Fail");
   }
*/

static void report(
        const uint16_t value, const char * label, const char * delim="   ")
{
    Serial.print(label);
    Serial.print(value);
    Serial.print(delim);
}


void setup()
{
    // Set up serial debugging
    Serial.begin(115200);

    // Start incoming SBUS connection from TX
    _sbus.Begin();

    // Start ESP-NOW
    //_esp.begin();

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    //esp_now_register_send_cb(onDataSent);
}

void loop()
{
    if (_sbus.Read()) {

        const auto data = _sbus.data();

        Serial.printf("%d\n", data.ch[0]);

        //_serializer.serializeRawRc(200, c1, c2, c3, c4, c5, c6);

        //_esp.send(_serializer.outBuf, _serializer.outBufSize);
    }

    // delay(5);
}
