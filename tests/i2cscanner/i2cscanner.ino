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

#include <bootloader.hpp>

static const uint8_t LED_PIN = 13;

static const uint8_t SDA_PIN = 0;
static const uint8_t SCL_PIN = 1;

//static TwoWire wire1 = TwoWire(SDA_PIN, SCL_PIN);

void serialEvent()
{
    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}


void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);

    Wire.begin();
    //wire1.begin();

    delay(100);
}

static void scan(TwoWire & wire, const char * name)
{

    Serial.print("Scanning ");
    Serial.print(name);
    Serial.println (" ...");

    int nDevices = 0;

    for(byte address = 1; address < 127; address++ ) {

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

    Serial.println(nDevices == 0 ? "No I2C devices found\n" : "done\n");

    delay(1000);

    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);

}


void loop()
{  
    scan(Wire, "Wire");
    //scan(wire1, "wire1");
}
