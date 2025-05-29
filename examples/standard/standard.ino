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

#include <BMI088.h>
#include <VL53L1X.h>

#include <system.h>
#include <tasks/imu.hpp>
#include "tasks/zranger2.hpp"

static const uint8_t GYRO_INTERRUPT_PIN = 4;
static const uint8_t LED_PIN = 5;
static const uint8_t FLOWDECK_CS_PIN = 10;

static Bmi088Accel _accel(Wire, 0x19);
static Bmi088Gyro _gyro(Wire, 0x69);

static VL53L1X _rangefinder;

static ImuTask * _imuTask;

static volatile uint32_t _gyro_count;

static void gyro_drdy()
{
    if (_imuTask) {
        _imuTask->dataAvailableCallback();
    }

    _gyro_count++;
}

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);

    Wire1.begin();
    Wire1.setClock(400000);
    _rangefinder.setBus(&Wire1);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    systemInit(LED_PIN, FLOWDECK_CS_PIN);
}

void loop()
{
}

// ImuTask -------------------------------------------------------------------

void ImuTask::deviceInit(void)
{
    _imuTask = this;

    if (!_accel.begin()) {
        systemReportForever("Unable to start accel");
    }

    if (!_gyro.begin()) {
        systemReportForever("Unable to start accel");
    }

    _accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
    _accel.setRange(Bmi088Accel::RANGE_24G);

    _gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    _gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
    _gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    _gyro.mapDrdyInt3(true);

    pinMode(4, arduino::INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN, gyro_drdy,  arduino::RISING);  
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    dataOut->x = _accel.getAccelX_raw();
    dataOut->y = _accel.getAccelY_raw();
    dataOut->z = _accel.getAccelZ_raw();

    Serial.println(_gyro_count);
}

void ImuTask::readGyroRaw(Axis3i16 * dataOut)
{
}


// ZRangerTask ---------------------------------------------------------------

void ZRangerTask::hardware_init()
{ 
    if (!_rangefinder.init()) {
        systemReportForever("VL53L1X::init() failed");
    }

    _rangefinder.setDistanceMode(VL53L1X::Long);
    _rangefinder.setMeasurementTimingBudget(50000);
    _rangefinder.startContinuous(50);
}

float ZRangerTask::hardware_read()
{
    return (float)_rangefinder.read();
}

// Debugging  ---------------------------------------------------------------

void debug(const char * msg)
{
    Serial.println(msg);
}
