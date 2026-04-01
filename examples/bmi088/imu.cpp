/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <BMI088.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/imu/device_api.h>
using namespace hf;

Bmi088Accel _accel(Wire, 0x19);

Bmi088Gyro _gyro(Wire, 0x69);

static constexpr uint8_t GRANGE = Bmi088Gyro::RANGE_2000DPS;
static constexpr uint8_t ARANGE = Bmi088Accel::RANGE_24G;

static void checkStatus(const int status, const char * attempt)
{
    while (status < 0) {
        printf("Failed to %s\n", attempt);
        delay(500);
    }
}

void IMU::begin() 
{
    checkStatus(_gyro.begin(), "initialize gyro");
    checkStatus(_gyro.setRange(Bmi088Gyro::RANGE_2000DPS), "set gyro range");

    checkStatus(_accel.begin(), "initialize accel");
    checkStatus(_accel.setRange(Bmi088Accel::RANGE_24G), "set accel range");
}

int16_t IMU::gyroRangeDps()
{
    static constexpr int16_t granges[5] = {2000, 1000, 500, 250, 125};

    return granges[GRANGE];
}

int16_t IMU::accelRangeGs()
{
    static constexpr int16_t aranges[4] = {3, 6, 12, 24};

    return aranges[ARANGE];
}

ImuRaw IMU::read()
{
    _gyro.readSensor();

    _accel.readSensor();

    return ImuRaw(

            ThreeAxisRaw(
                _gyro.getGyroX_raw(),
                _gyro.getGyroY_raw(),
                _gyro.getGyroZ_raw()),

            ThreeAxisRaw(
                _accel.getAccelX_raw(),
                _accel.getAccelY_raw(),
                _accel.getAccelZ_raw()));
}
