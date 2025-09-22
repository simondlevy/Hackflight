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

#include <Wire.h>   

static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static TwoWire wire = TwoWire(SDA_PIN, SCL_PIN);

void setup()
{
    Serial.begin(115200);

    wire.begin();

    delay(100);
}

void loop()
{  
    Serial.println("Scanning ...");

    int nDevices = 0;

    for(byte address = 1; address < 127; address++ ) 
    {

        wire.beginTransmission(address);

        byte error = wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);

            nDevices++;
        }
        else if (error==4) 
        {
            Serial.print("Unknown error at address 0x");
            if (address<16) 
                Serial.print("0");
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n"); 

    delay(1000);
}
