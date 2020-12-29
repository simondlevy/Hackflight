/*
   Test TinyPICO running DSHOT600 ESC protocol


   Copyright (c) 2020 Simon D. Levy

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

static const uint8_t PINS[2] = {15, 25};

hf::Esp32DShot600 motors = hf::Esp32DShot600(PINS, 2);

static uint8_t state;

void setup(void)
{
    Serial.begin(115200);
    motors.init();

    state = 0;
}

void loop(void)
{
    switch (state) {

        case 0:  
            Serial.println("Hit Enter to arm");
            if (Serial.available()) {
                Serial.read();
                motors.arm();
                state = 1;
            }
            delay(1000);
            break;

        case 1: 
            Serial.println("Hit Enter to start motor");
            if (Serial.available()) {
                Serial.read();
                motors.write(0, 0.1);
                motors.write(1, 0.1);
                state = 2;
            }
            delay(1000);
            break;

        case 2: 
            Serial.println("Hit Enter to stop motor");
            if (Serial.available()) {
                Serial.read();
                motors.write(0, 0);
                motors.write(1, 0);
                state = 3;
            }
            delay(1000);
            break;

        case 3: 
            Serial.println("Hit Enter disarm");
            if (Serial.available()) {
                Serial.read();
                motors.disarm();
                state = 0;
            }
            delay(1000);
            break;

        default:
            break;

    }
}
