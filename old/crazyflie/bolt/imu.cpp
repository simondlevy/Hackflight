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

#include <SPI.h>

#include <BMI088.h>

#include <tasks/imu.hpp>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;
static const uint8_t GYRO_INT_PIN = PC14;

static const uint8_t MISO_PIN = PB14;
static const uint8_t MOSI_PIN = PB15;
static const uint8_t SCLK_PIN = PB13;

static SPIClass spi;

static Bmi088Accel accel(spi, ACCEL_CS_PIN);

static Bmi088Gyro gyro(spi, GYRO_CS_PIN);

static ImuTask * imuTask;

static void handle_gyro_interrupt()
{
    imuTask->dataAvailableCallback();
}

static bool failed(const int status)
{
    return status < 0;
}

bool ImuTask::device_init()
{
    imuTask = this;

    spi.setMISO(MISO_PIN);
    spi.setMOSI(MOSI_PIN);
    spi.setSCLK(SCLK_PIN);

    if (failed(gyro.begin())) return false;

    if (failed(accel.begin())) return false;

    if (failed(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ))) return false;

    if (failed(gyro.setRange(Bmi088Gyro::RANGE_2000DPS))) return false;

    if (failed(gyro.pinModeInt3(
                        Bmi088Gyro::PIN_MODE_PUSH_PULL,
                        Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)))
        return false;
            
    if (failed(gyro.mapDrdyInt3(true))) return false;

    if (failed(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ))) return false;

    if (failed(accel.setRange(Bmi088Accel::RANGE_24G))) return false;

    pinMode(GYRO_INT_PIN, INPUT);
    attachInterrupt(GYRO_INT_PIN, handle_gyro_interrupt, RISING);

    return true;
}

void ImuTask::device_readRaw(
        int16_t & gx, int16_t & gy, int16_t & gz, 
        int16_t & ax, int16_t & ay, int16_t & az)
{
    gyro.readSensor();

    gx = gyro.getGyroX_raw();
    gy = gyro.getGyroY_raw();
    gz = gyro.getGyroZ_raw();

    accel.readSensor();

    ax = accel.getAccelX_raw();
    ay = accel.getAccelY_raw();
    az = accel.getAccelZ_raw();
}

float ImuTask::gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 24 / 65536.f;
}
