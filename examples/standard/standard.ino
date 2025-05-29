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

#include <arduino_freertos.h>
#include <avr/pgmspace.h>

#include <VL53L1X.h>

#include <system.h>
#include <tasks/imu.hpp>
#include "tasks/zranger2.hpp"

static const uint8_t LED_PIN = 5;
static const uint8_t FLOWDECK_CS_PIN = 10;

static VL53L1X _vl53l1x;

static void reportForever(const char *msg)
{
    while (true) {
        printf("%s\n", msg);
        delay(500);
    }
}

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);

    Wire1.begin();
    Wire1.setClock(400000);
    _vl53l1x.setBus(&Wire1);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    systemInit(LED_PIN, FLOWDECK_CS_PIN);
}

void loop() {}

// ImuTask -------------------------------------------------------------------

void ImuTask::deviceInit(void)
{
}

void ImuTask::readGyroRaw(Axis3i16 * dataOut)
{
    (void)dataOut;
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    (void)dataOut;
}


// ZRangerTask ---------------------------------------------------------------

void ZRangerTask::hardware_init()
{ 
    if (!_vl53l1x.init()) {
        reportForever("VL53L1X::init() failed");
    }

    _vl53l1x.setDistanceMode(VL53L1X::Long);
    _vl53l1x.setMeasurementTimingBudget(50000);
    _vl53l1x.startContinuous(50);
}

float ZRangerTask::hardware_read()
{
    return (float)_vl53l1x.read();
}

// Debugging  ---------------------------------------------------------------

void debug(const char * msg)
{
    Serial.println(msg);
}
