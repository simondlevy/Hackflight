/*
   TestInterrupt.cpp : serial-interrupt test

   Copyright (C) Simon D. Levy 2018

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

static uint8_t buf[100];
static uint8_t bufcount;

void serialEvent3()
{
    uint32_t time = micros();
    static uint32_t _time;

    /*
    static bool onoff;
    if (time-_time > 500000) {
        _time = time;
        digitalWrite(4, onoff?LOW:HIGH);
        onoff = !onoff;
    }*/

    if (time-_time > 2500) {
        bufcount = 0;
    }

    buf[bufcount++] = Serial3.read();

    _time = time;
}


void setup() {
  
  Serial.begin(115200);

  Serial3.begin(115200);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}

void loop() {

    if (bufcount == 16) {

        for (int k=0; k<16; ++k) {
            Serial.printf("%x\n", buf[k]);
        }
    }

    delay(5);

}
