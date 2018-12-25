/*
   LoiterBoard.ino : Sketch allowing Butterfly breakout board to provide
   optical flow and Above-Ground-Level messages to a flight controller

   Additional libraries required:

      https://github.com/simondlevy/CrossPlatformDataBus
      https://github.com/simondlevy/VL53L1X
      https://github.com/simondlevy/PMW3901

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modiflowy
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>

#include <VL53L1X.h>
#include <PMW3901.h>

#include "mspparser.hpp"

// Set this to Serial for USB debugging, or Serial1 for depolyment
static HardwareSerial * UART = &Serial;

static uint16_t FLOW_UPDATE_HZ = 20;

static const uint8_t VCC_PIN = A0;
static const uint8_t GND_PIN = A1;

static const uint8_t CS_PIN  = 10;

static VL53L1X distanceSensor;

static PMW3901 flowSensor(CS_PIN);

static uint32_t _flowUpdateMicros = 1000000 / FLOW_UPDATE_HZ;

static void powerPin(uint8_t pin, uint8_t value)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
}

static void error(const char * sensorName)
{
    while (true) {
        Serial.print(sensorName);
        Serial.println(" offline!");
    }
}

void setup(void)
{
    // Use digital pins to power VL53L1X
    powerPin(GND_PIN, LOW);
    powerPin(VCC_PIN, HIGH);

    // Start I^2C bus
    Wire.begin(TWI_PINS_6_7);
    delay(100); // Wait a bit for bus to start

    // Start serial comms
    UART->begin(115200);

    // Start VL53L1X distance sensor
    if (!distanceSensor.begin()) {
        error("VL53L1X");
    }

    // Start PMW3901 optical-flow sensor
    if (!flowSensor.begin()) {
        error("PMW3901");
    }
}

void loop(void)
{
    // Declare measurement variables static so they'll persist between calls to loop()
    static int16_t range, flowx, flowy;

    // Read distance sensor when its data is available
    if (distanceSensor.newDataReady()) {
        range = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    }

    // Read flow sensor periodically
    static uint32_t _time;
    uint32_t time = micros();
    if (time-_time > _flowUpdateMicros) {
        flowSensor.readMotionCount(&flowx, &flowy);
        _time = time;
        uint8_t msg[hf::MspParser::MAXMSG];
        uint8_t len = hf::MspParser::serialize_SET_RANGE_AND_FLOW(msg, range, flowx, flowy);
        for (uint8_t k=0; k<len; ++k) {
            UART->write(msg[k]);
        }
    }
}
