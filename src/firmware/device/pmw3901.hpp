/*
   Copyright (C) 2026 Simon D. Levy

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

#include <pmw3901.hpp>

#include <SPI.h>

#if 0
// Using digital pin 10 for chip select
static PMW3901 sensor;

void setup() 
{
    Serial.begin(115200);

    SPI.begin();

    if (!sensor.begin()) {

        while(true) { 
            Serial.println("Initialization of the flow sensor failed");
            delay(500);
        }
    }
}

void loop() 
{
    int16_t deltaX = 0;
    int16_t deltaY = 0;
    bool gotMotion = false;

    sensor.readMotion(deltaX, deltaY, gotMotion); 

    Serial.print("deltaX: ");
    Serial.print(deltaX);
    Serial.print(",\tdeltaY: ");
    Serial.print(deltaY);
    Serial.print(",\tgotMotion: ");
    Serial.println(gotMotion ? "yes" : "no");

    delay(100);
}
#endif
