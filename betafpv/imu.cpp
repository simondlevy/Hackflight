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

#include <mpu6x00.hpp>

#include <tasks/imu.hpp>

static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;

static const uint8_t CS_PIN  = PA4;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static Mpu6000 imu = Mpu6000(spi, CS_PIN);

bool ImuTask::device_init()
{
    spi.begin();

    if (!imu.begin()) return false;

    return true;
}

void ImuTask::device_readRaw(
        int16_t & gx, int16_t & gy, int16_t & gz, 
        int16_t & ax, int16_t & ay, int16_t & az)
{
        imu.readSensor();

        gx = imu.getRawGyroX();
        gy = imu.getRawGyroY();
        gz = imu.getRawGyroZ();

        ax = imu.getRawAccelX();
        ay = imu.getRawAccelY();
        az = imu.getRawAccelZ();
}

float ImuTask::device_gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::device_accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 16 / 65536.f;
}
