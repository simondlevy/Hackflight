/*
   Hackflight embedded SNN example

   Copyright (C) 2024 Simon D. Levy

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

#include <Wire.h>
#include <VL53L1X.h>

#include <embedded_snn.hpp>

static VL53L1X sensor;

static float reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void setup() 
{
    // Support serial debugging
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    sensor.setBus(&Wire);

    sensor.setTimeout(500);

    if (!sensor.init()) {
        reportForever("Failed to detect and initialize sensor!");
    }

    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);

    // Set up two LEDs
    pinMode(2, OUTPUT);
    pinMode(22, OUTPUT);

    EmbeddedSnn::load();
}

void loop() 
{
    const double observations[1]={ (double)sensor.read() / 10 }; // mm => cm 

    if (sensor.timeoutOccurred()) {
        reportForever("Sensor timed out");
    }

    double actions[2] = {};

    EmbeddedSnn::step(observations, actions);

    digitalWrite(2, actions[0] > actions[1] ? HIGH : LOW);
    digitalWrite(22, actions[0] > actions[1] ? LOW : HIGH);
}
