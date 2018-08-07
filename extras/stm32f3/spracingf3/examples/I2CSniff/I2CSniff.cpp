/*
   I2CSniff sniff and report I^2C devices 

   Copyright (C) 2016 Simon D. Levy 

   Don't forget to supply external power for external sensors (like MB1242 sonar)!

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

void setup(void)
{
    Serial.begin(115200);
    Wire.begin(1);  // Note 1!
} 

void loop(void)
{
    uint8_t addr;

    for (addr=0; addr<128; ++addr) {
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
            Serial.printf("Found device at address 0X%02X\n", addr);
        }
    }

    Serial.printf("--------------------------\n");
}
