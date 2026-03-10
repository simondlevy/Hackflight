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

    class SixAxis {

        public:

            Vec3 gyro;
            Vec3 accel;

            SixAxis() = default;

            SixAxis(const Vec3 & gyro, const Vec3 & accel)
                : gyro(gyro), accel(accel) {}

            SixAxis& operator=(const SixAxis& other) = default;
      };

    class ImuRaw {

        public:

            int16_t gx;
            int16_t gy;
            int16_t gz;
            int16_t ax;
            int16_t ay;
            int16_t az;

            ImuRaw() = default;

            ImuRaw(
                    const int16_t gx, const int16_t gy, const int16_t gz,
                    const int16_t ax, const int16_t ay, const int16_t az)
                : gx(gx), gy(gy), gz(gz), ax(ax), ay(ay), az(az) {}

            ImuRaw& operator=(const ImuRaw& other) = default;
     };

    class ImuFiltered {

        public:

            // Degrees per second
            float gx;
            float gy;
            float gz;

            // Gs
            float ax;
            float ay;
            float az;

            ImuFiltered() = default;

            ImuFiltered(
                    const float gx, const float gy, const float gz,
                    const float ax, const float ay, const float az)
                : gx(gx), gy(gy), gz(gz), ax(ax), ay(ay), az(az) {}

            ImuFiltered& operator=(const ImuFiltered& other) = default;
     };
}
