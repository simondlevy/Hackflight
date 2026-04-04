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

namespace hf {

    class IMU {

        public:

            class ThreeAxisRaw {

                public:

                    int16_t x;
                    int16_t y;
                    int16_t z;

                    ThreeAxisRaw() = default;

                    ThreeAxisRaw(const int16_t x, const int16_t y, const int16_t z) 
                        : x(x), y(y), z(z) {}

                    ThreeAxisRaw& operator=(const ThreeAxisRaw& other) = default;
            };

            class RawData {

                public:

                    ThreeAxisRaw gyro;
                    ThreeAxisRaw accel;

                    RawData() = default;

                    RawData(const ThreeAxisRaw & gyro, const ThreeAxisRaw & accel)
                        : gyro(gyro), accel(accel) {}

                    RawData& operator=(const RawData& other) = default;
            };

            class ThreeAxis {

                public:

                    float x;
                    float y;
                    float z;

                    ThreeAxis() = default;

                    ThreeAxis(const float x, const float y, const float z) 
                        : x(x), y(y), z(z) {}

                    ThreeAxis& operator=(const ThreeAxis& other) = default;

                    ThreeAxis operator+(const ThreeAxis& other) const
                    {
                        return ThreeAxis(x+other.x, y+other.y, z+other.z);
                    }

                    ThreeAxis operator-(const ThreeAxis& other) const
                    {
                        return ThreeAxis(x-other.x, y-other.y, z-other.z);
                    }

                    ThreeAxis operator*(const ThreeAxis& other) const
                    {
                        return ThreeAxis(x*other.x, y*other.y, z*other.z);
                    }

                    ThreeAxis operator*(const float v) const
                    {
                        return ThreeAxis(x*v, y*v, z*v);
                    }

                    ThreeAxis operator/(const float d) const
                    {
                        return ThreeAxis(x/d, y/d, z/d);
                    }

                    bool operator<(const float v) const
                    {
                        return x < v && y < v && z < v;
                    }
            };

            class FilteredData {

                public:

                    ThreeAxis gyroDps;
                    ThreeAxis accelGs;

                    FilteredData() = default;

                    FilteredData(const ThreeAxis & gyroDps, const ThreeAxis & accelGs) 
                        : gyroDps(gyroDps), accelGs(accelGs) {}

                    FilteredData& operator=(const FilteredData& other) = default;
            };

            auto begin() -> bool;

            auto gyroRangeDps() -> int16_t;

            auto accelRangeGs() -> int16_t;

            auto read() -> RawData;
    };
}
