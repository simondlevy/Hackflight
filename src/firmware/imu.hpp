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

#pragma once

#include <hackflight.h>
#include <firmware/datatypes.hpp>

namespace hf {

    class IMU {

        public:

            class RawData {

                public:

                    ThreeAxisRaw gyro;
                    ThreeAxisRaw accel;

                    RawData() = default;

                    RawData(const ThreeAxisRaw & gyro, const ThreeAxisRaw & accel)
                        : gyro(gyro), accel(accel) {}

                    RawData& operator=(const RawData& other) = default;
            };

            auto begin() -> bool;

            auto gyroRangeDps() -> int16_t;

            auto accelRangeGs() -> int16_t;

            auto read() -> RawData;
    };
}
