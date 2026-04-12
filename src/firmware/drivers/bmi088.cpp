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

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <BMI088.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/imu/sensor.hpp>
using namespace hf;

static constexpr Bmi088Gyro::Range GRANGE = Bmi088Gyro::RANGE_2000DPS;

static constexpr Bmi088Accel::Range ARANGE = Bmi088Accel::RANGE_24G;

// The SDO pin should either be pulled low for the 0x18/0x68
// addresses, high for 0x19/0x69
static Bmi088Accel _accel = Bmi088Accel(Wire, 0x18);
static Bmi088Gyro _gyro = Bmi088Gyro(Wire, 0x68);

static bool okay(const int status)
{
    return status >= 0;
}

auto IMU::begin() -> bool
{
    return 

        okay(_gyro.begin()) &&

        okay(_accel.begin()) &&

        okay(_gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

        okay(_gyro.setRange(GRANGE)) &&

        okay(_gyro.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

        okay(_gyro.mapDrdyInt3(true)) &&

        okay(_accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

        okay(_accel.setRange(ARANGE));
}

auto IMU::gyroRangeDps() -> int16_t
{
    static constexpr int16_t granges[5] = {2000, 1000, 500, 250, 125};

    return granges[GRANGE];
}

auto IMU::accelRangeGs() -> int16_t
{
    static constexpr int16_t aranges[4] = {3, 6, 12, 24};

    return aranges[ARANGE];
}


auto IMU::read() -> IMU::RawData
{
    _gyro.readSensor();

    _accel.readSensor();

    return IMU::RawData(
            IMU::ThreeAxisRaw(
                _gyro.getGyroX_raw(),
                _gyro.getGyroY_raw(),
                _gyro.getGyroZ_raw()
                ),
            IMU::ThreeAxisRaw(
                _accel.getAccelX_raw(),
                _accel.getAccelY_raw(),
                _accel.getAccelZ_raw()
                ));
}
