/*
   Demo sketch for DSMRX library

   Displays channel values in interval [-1,+1]

   Copyright (C) Simon D. Levy 2017

   This file is part of DSMRX.

   DSMRX is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   DSMRX is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with DSMRX.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <oneshot125.hpp>
#include <dsmrx.hpp>

static const uint8_t CHANNELS = 6;

static const std::vector<uint8_t> PINS = {5};

static Dsm2048 rx;

static auto motors = OneShot125(PINS);

void serialEvent3(void)
{
    while (Serial3.available()) {
        rx.parse(Serial3.read(), micros());
    }
}

static void blinkLed()
{
    static uint32_t msec_prev;
    const auto msec_curr = millis();
    static bool on;

    if (msec_curr - msec_prev > 500) {
        digitalWrite(LED_BUILTIN, on);
        on = !on;
        msec_prev = msec_curr;
    }
}


void setup(void)
{
    Serial.begin(115000);

    Serial3.begin(115000);

    pinMode(LED_BUILTIN, OUTPUT);

    motors.arm(); 
}

void loop(void)
{
    blinkLed();

    static float throttle;

    if (rx.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float values[CHANNELS];

        rx.getChannelValuesMlp6Dsm(values);

        throttle = values[0];
    }

    Serial.println(throttle);
}
