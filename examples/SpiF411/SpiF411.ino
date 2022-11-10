/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */


#include <SPI.h>

static const uint8_t MOSI_PIN = PA7;  
static const uint8_t MISO_PIN = PA6;  
static const uint8_t SCLK_PIN = PA5;  
static const uint8_t CS_PIN   = PA4;
static const uint8_t LED_PIN  = PC14;

static SPIClass _spi(MOSI_PIN, MISO_PIN, SCLK_PIN);

void setup(void)
{
    Serial.begin(115200);

    _spi.begin();

    pinMode(LED_PIN, OUTPUT);
}

void loop(void)
{
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);

    Serial.println(millis());
}
