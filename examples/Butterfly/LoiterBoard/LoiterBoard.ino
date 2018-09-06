/*
   LoiterBoard.ino : Sketch allowing Butterfly breakout board to provide
   optical flow and above-ground-level messages to a flight controller

   Additional libraries required:

      https://github.com/simondlevy/CrossPlatformDataBus
      https://github.com/simondlevy/VL53L1X
      https://github.com/bitcraze/Bitcraze_PMW3901

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>

#include <VL53L1X.h>
#include <Bitcraze_PMW3901.h>

#include "hackflight.hpp"

static const uint8_t VCC_PIN = 8;
static const uint8_t GND_PIN = 9;

static VL53L1X _distanceSensor;

static void powerPin(uint8_t pin, uint8_t value)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
}

void setup(void)
{
    // Use digital pins for VL53L1 power, ground
    powerPin(GND_PIN, LOW);
    powerPin(VCC_PIN, HIGH);

    Serial.begin(115200);

    Wire.begin();

    delay(100);

    _distanceSensor.begin();
}

void loop(void)
{
    if (_distanceSensor.newDataReady()) {

        float distance = _distanceSensor.getDistance() / 1000.f; // mm => m

        Serial.println(distance);
    }
}
