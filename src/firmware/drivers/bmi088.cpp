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

static constexpr Bmi088Gyro::Range KGyroRange = Bmi088Gyro::RANGE_2000DPS;

static constexpr Bmi088Accel::Range kAccelRange = Bmi088Accel::RANGE_24G;

// The SDO pin should either be pulled low for the 0x18/0x68
// addresses, high for 0x19/0x69
static Bmi088Accel accel_ = Bmi088Accel(Wire, 0x18);
static Bmi088Gyro gyro_ = Bmi088Gyro(Wire, 0x68);

static bool okay(const int status)
{
    return status >= 0;
}

namespace hf {

    auto IMU::Begin() -> bool
    {
        return 

            okay(gyro_.begin()) &&

            okay(accel_.begin()) &&

            okay(gyro_.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

            okay(gyro_.setRange(KGyroRange)) &&

            okay(gyro_.pinModeInt3(
                        Bmi088Gyro::PIN_MODE_PUSH_PULL,
                        Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

            okay(gyro_.mapDrdyInt3(true)) &&

            okay(accel_.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

            okay(accel_.setRange(kAccelRange));
    }

    auto IMU::GetGyroRangeDps() -> int16_t
    {
        static constexpr int16_t granges[5] = {2000, 1000, 500, 250, 125};

        return granges[KGyroRange];
    }

    auto IMU::GetAccelRangeGs() -> int16_t
    {
        static constexpr int16_t aranges[4] = {3, 6, 12, 24};

        return aranges[kAccelRange];
    }


    auto IMU::Read() -> IMU::RawData
    {
        gyro_.readSensor();

        accel_.readSensor();

        return IMU::RawData(
                IMU::ThreeAxisRaw(
                    gyro_.getGyroX_raw(),
                    gyro_.getGyroY_raw(),
                    gyro_.getGyroZ_raw()
                    ),
                IMU::ThreeAxisRaw(
                    accel_.getAccelX_raw(),
                    accel_.getAccelY_raw(),
                    accel_.getAccelZ_raw()
                    ));
    }

} // namespace hf
