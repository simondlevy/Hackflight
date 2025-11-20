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
#include <ICM42688.h>

#include <imu.hpp>

static SPIClass spi = SPIClass(PA7, PA6, PA5);

static ICM42688 icm42688(spi, PB12);

static bool failed(const int status)
{
    return status < 0;
}

bool Imu::device_init(int16_t & gscale, int16_t & ascale)
{
    if (failed(icm42688.begin())) return false;

    gscale = 2000;
    ascale = 24;

    return true;
}

void Imu::device_read(
        int16_t & gx, int16_t & gy, int16_t & gz,
        int16_t & ax, int16_t & ay, int16_t & az)
{
    icm42688.getRawAGT();

    gx = icm42688.rawGyrX();
    gy = icm42688.rawGyrY();
    gz = icm42688.rawGyrZ();

    ax = icm42688.rawAccX();
    ay = icm42688.rawAccY();
    az = icm42688.rawAccZ();
}
