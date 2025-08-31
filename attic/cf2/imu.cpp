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

#include <tasks/imu.hpp>

#include <BMI088.h>

// internal I^2C
static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t GYRO_INTERRUPT_PIN = PC14;

static const uint8_t ACCEL_ADDRESS = 0x18;
static const uint8_t GYRO_ADDRESS = 0x69;

static TwoWire wire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDRESS);

static Bmi088Gyro gyro(wire, GYRO_ADDRESS);

static ImuTask * imuTask;

static void gyro_drdy()
{
    imuTask->dataAvailableCallback();
}

static void check(const int status, const char * msg)
{
    if (status < 0) {
        Serial.println(msg);
        while (true) ;
    }
}

static void add_interrupt(const uint8_t pin, void (*handler)())
{
    pinMode(pin, INPUT);
    attachInterrupt(pin, handler, RISING);
}

void ImuTask::deviceInit(void)
{
    imuTask = this;

    check(gyro.begin(), "Gyro Initialization Error");

    gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    gyro.setRange(Bmi088Gyro::RANGE_2000DPS);

    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    add_interrupt(GYRO_INTERRUPT_PIN, gyro_drdy);

    check(accel.begin(), "Accel Initialization Error");

    accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
    accel.setRange(Bmi088Accel::RANGE_24G);
}

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
    dataOut->x = gyro.getGyroX_raw();
    dataOut->y = gyro.getGyroY_raw();
    dataOut->z = gyro.getGyroZ_raw();
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    dataOut->x = accel.getAccelX_raw();
    dataOut->y = accel.getAccelY_raw();
    dataOut->z = accel.getAccelZ_raw();
}
