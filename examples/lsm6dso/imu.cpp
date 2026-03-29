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

#include <LSM6DSOSensor.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/sensors/imu.hpp>
using namespace hf;

static constexpr int16_t GODR = 1000;
static constexpr int16_t GRANGE = 2000;
static constexpr int16_t ARANGE = 16;

LSM6DSOSensor _lsm6dso = LSM6DSOSensor(&Wire);

static bool bad(const LSM6DSOStatusTypeDef status)
{
    return status != LSM6DSO_OK;
}

void IMU::begin() 
{
    Wire.begin();

    Wire.setClock(1000000);

    uint8_t id = 0;
    _lsm6dso.ReadID(&id);

    if (id != LSM6DSO_ID) {
        Debugger::reportForever("LSM6DSO not detected");
    }

    if (
            bad(_lsm6dso.begin())  ||
            bad(_lsm6dso.Enable_G())  ||
            bad(_lsm6dso.Enable_X())  ||
            bad(_lsm6dso.Set_G_ODR(GODR)) ||
            bad(_lsm6dso.Set_X_FS(ARANGE)) ||
            bad(_lsm6dso.Set_G_FS(GRANGE)))
    {
        Debugger::reportForever(
                "LSM6DSO initialization unsuccessful");
    }
}

int16_t IMU::gyroRangeDps()
{
    return GRANGE;
}

int16_t IMU::accelRangeGs()
{
    return ARANGE;
}

ImuRaw IMU::read()
{
    int16_t gyro[3] = {};
    _lsm6dso.Get_G_AxesRaw(gyro);

    int16_t accel[3] = {};
    _lsm6dso.Get_X_AxesRaw(accel);

    // Rotation 90 degrees counter-clockwise
    return ImuRaw(
            hf::ThreeAxisRaw(-gyro[1], gyro[0], gyro[2]),
            hf::ThreeAxisRaw(-accel[1], accel[0], accel[2]));
}
