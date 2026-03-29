/*
   Hackflight main sketch

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

// Hackflight library
#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/sensors/imus/mpu6050.hpp>
#include <firmware/sensors/imus/wire1_lsm6dso_rot90ccw.hpp>

static hf::IMU _mpu6050;
static hf::NewIMU _lsm6dso;

void setup()
{
    _mpu6050.begin();

    _lsm6dso.begin();

}

void loop()
{
    if (_mpu6050.available()) {
        const auto mpu6050_raw = _mpu6050.read();
        hf::Debugger::report(mpu6050_raw);
    }

    if (_lsm6dso.available()) {
        const auto lsm6dso_raw = _lsm6dso.read();
        hf::Debugger::report(lsm6dso_raw);
    }
}
