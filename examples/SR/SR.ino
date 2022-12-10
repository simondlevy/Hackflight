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

#include <VL53L5cx.h>

//#include <hackflight.h>
//#include <msp/parser.h>
//#include <espnow.h>
//#include <esp_now.h>


static const uint8_t VL53L5_INT_PIN = 4; // Set to 0 for polling
static const uint8_t VL53L5_LPN_PIN =  14;

// Set to 0 for continuous mode
static const uint8_t VL53L5_INTEGRAL_TIME_MS = 10;

static VL53L5cx _vl53l5(VL53L5_LPN_PIN, VL53L5_INTEGRAL_TIME_MS, VL53L5cx::RES_4X4_HZ_1);


//static const bool UART_INVERTED = false;
//static const uint8_t RX_PIN = 4; // unused
//static const uint8_t TX_PIN = 14;

//static MspSerializer _serializer;

void setup()
{
    Serial.begin(115200);

    // Start outgoing serial connection to FC, inverted
    //Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN, UART_INVERTED);
}

void loop()
{
    //static uint8_t _count;
    //Serial1.write(_count);
    //_count = (_count + 1) % 256;

    delay(5);
}
