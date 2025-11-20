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

#include <imu.hpp>

#ifdef BOLT

static const uint8_t ACCEL_CS_PIN = PB1;
static const uint8_t GYRO_CS_PIN = PB0;

static const uint8_t MISO_PIN = PB14;
static const uint8_t MOSI_PIN = PB15;
static const uint8_t SCLK_PIN = PB13;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static Bmi088Accel accel(spi, ACCEL_CS_PIN);

static Bmi088Gyro gyro(spi, GYRO_CS_PIN);

#else

static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t ACCEL_ADDR = 0x18;
static const uint8_t GYRO_ADDR = 0x69;

static TwoWire wire = TwoWire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDR);

static Bmi088Gyro gyro(wire, GYRO_ADDR);

#endif

static bool okay(const int status)
{
    return status >= 0;
}

bool Imu::device_init(int16_t & gscale, int16_t & ascale)
{
    gscale = 2000;
    ascale = 24;

    return 

        okay(gyro.begin()) &&

        okay(accel.begin()) &&

        okay(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

        okay(gyro.setRange(Bmi088Gyro::RANGE_2000DPS)) &&

        okay(gyro.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

        okay(gyro.mapDrdyInt3(true)) &&

        okay(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

        okay(accel.setRange(Bmi088Accel::RANGE_24G));
}

void Imu::device_read(
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
