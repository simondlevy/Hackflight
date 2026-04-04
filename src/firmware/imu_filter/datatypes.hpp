/**
 * Copyright (C) 2026 Simon D. Levy
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

#pragma once

#include <firmware/datatypes.hpp>

namespace hf {

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

    class ThreeAxisStats {

        public:

            ThreeAxis mean;
            ThreeAxis variance;

            ThreeAxisStats() = default;

            ThreeAxisStats
                (const ThreeAxis& mean, const ThreeAxis& variance) 
                : mean(mean), variance(variance) {}

            ThreeAxisStats& operator=(const ThreeAxisStats& other) = default;
    };

    class ImuFiltered {

        public:

            ThreeAxis gyroDps;
            ThreeAxis accelGs;

            ImuFiltered() = default;

            ImuFiltered(const ThreeAxis & gyroDps, const ThreeAxis & accelGs) 
                : gyroDps(gyroDps), accelGs(accelGs) {}

            ImuFiltered& operator=(const ImuFiltered& other) = default;
    };
}
