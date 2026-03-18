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

#include <stdint.h>

#include <datatypes.hpp>

namespace hf {

    class RxData {

        public:

            Setpoint axes;
            float aux;
            bool is_armed;
            bool is_throttle_down;

            RxData() = default;

            RxData(
                    const Setpoint & axes,
                    const float aux,
                    const bool is_armed,
                    const bool is_throttle_down)
                :
                    axes(axes),
                    aux(aux),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down) {}

            RxData& operator=(const RxData& other) = default;
    };

    class ThreeAxisStats {

        public:

            Vec3 mean;
            Vec3 variance;

            ThreeAxisStats() = default;

            ThreeAxisStats
                (const Vec3& mean, const Vec3& variance) 
                : mean(mean), variance(variance) {}

            ThreeAxisStats& operator=(const ThreeAxisStats& other) = default;
    };

    class ImuRaw {

        public:

            Vec3Raw gyro;
            Vec3Raw accel;

            ImuRaw() = default;

            ImuRaw(const Vec3Raw & gyro, const Vec3Raw & accel)
                : gyro(gyro), accel(accel) {}

            ImuRaw& operator=(const ImuRaw& other) = default;
    };

    class ImuFiltered {

        public:

            Vec3 gyroDps;
            Vec3 accelGs;

            ImuFiltered() = default;

            ImuFiltered(const Vec3 & gyroDps, const Vec3 & accelGs) 
                : gyroDps(gyroDps), accelGs(accelGs) {}

            ImuFiltered& operator=(const ImuFiltered& other) = default;
    };
}
