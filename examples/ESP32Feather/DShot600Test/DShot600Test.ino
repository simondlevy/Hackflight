/*
   Arduino sketch to test DSHOT600 protocol on ESP32 boards

   DID YOU REMEMOVE THE PROPELLERS FIRST?

   Copyright (c) 2019 Simon D. Levy

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

#include "hackflight.hpp"
#include "motors/esp32dshot600.hpp"

static float    val;
static int16_t  dir;
static uint8_t  sel;

static hf::Esp32DShot600 dshot;

void setup() {

    Serial.begin(115200);

    dshot.addMotor(4);
    dshot.addMotor(19);

    dshot.begin();

    // Turn LED on
    pinMode(13,OUTPUT);
    digitalWrite(13, HIGH);

    delay(5000); // allow both motors to start up

    // Turn LED off
    digitalWrite(13, LOW);

    sel = 0;
    val = 0;
    dir = +1;
}

void loop() {

    // spin active motor
    dshot.writeMotor(sel, val);

    // silence inactive motor
    dshot.writeMotor(1-sel, 0);

    // change the value slightly
    val += dir * .001;

    // stop halfway
    if (val >= 0.5) {
        dir = -1;
    }

    // at bottom end, switch direction and motor
    if (val <= 0) {
        dir = +1;
        sel = 1-sel;
    }

    delay(10);
}

