/**
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

#pragma once

#include <imu_api.h>

#include <SPI.h>

#include <BMI088.h>

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;

static const uint8_t GYRO_INT_PIN = PC14;

static SPIClass spi;

static Bmi088Accel accel(spi, ACCEL_CS_PIN);

static Bmi088Gyro gyro(spi, GYRO_CS_PIN);

static volatile uint32_t gyro_interrupt_count;

static void gyro_drdy()
{
    gyro_interrupt_count++;
}

bool imu_deviceInit()
{
    spi.setSCLK(PB13);
    spi.setMISO(PB14);
    spi.setMOSI(PB15);

    if (gyro.begin() < 0) {
        return false;
    }

    if (accel.begin() < 0) {
        return false;
    }

    gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    gyro.setRange(Bmi088Gyro::RANGE_2000DPS);

    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
    accel.setRange(Bmi088Accel::RANGE_24G);

    return true;
}

void imu_deviceReadRaw(
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

uint8_t imu_deviceGetInterruptPin()
{
    return GYRO_INT_PIN;
}

