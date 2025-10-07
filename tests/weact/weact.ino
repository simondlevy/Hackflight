/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

//static const uint8_t RX1_PIN = PA3;
//static const uint8_t TX1_PIN = PA2;
//static HardwareSerial serial1(RX1_PIN, TX1_PIN);


void setup() 
{
    Serial.begin(115200);

    Serial2.begin(115200);
}


void loop() 
{
    while (Serial2.available()) {
        Serial.println(Serial2.read());
    }
}
