/**
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Adafruit_NeoPixel.h>

#include <bootloader.hpp>

static const uint8_t PIN = 8;

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
static Adafruit_NeoPixel pixels(1, PIN, NEO_GRB + NEO_KHZ800);

void serialEvent()
{
    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}


void setup()
{
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}


void loop()
{  
    delay(500);
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    delay(500);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();

}
