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

#include <bootloaderJumper.hpp>

//static const uint8_t LED_PIN = PB5; // BetaFPV
static const uint8_t LED_PIN = PB2; // DIY

void serialEvent()
{
    if (Serial.available() && Serial.read() == 'R') {
        BootloaderJumper::jump();
    }
}

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}


void loop() 
{

    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);

}
