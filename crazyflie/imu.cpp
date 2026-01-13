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

// Arduino library
#include <BMI088.h>

// Hackflight
#include <firmware/imu.hpp>

#ifdef BOLT

static SPIClass spi = SPIClass(PB15, PB14, PB13);
static Bmi088Accel accel(spi, PB1);
static Bmi088Gyro gyro(spi, PB0);

#else

static TwoWire wire = TwoWire(PC9, PA8);
static Bmi088Accel accel(wire, 0x18);
static Bmi088Gyro gyro(wire, 0x69);

#endif

static bool okay(const int status)
{
    return status >= 0;
}

bool IMU::device_init(int16_t & gscale, int16_t & ascale)
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

void IMU::device_read(
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
