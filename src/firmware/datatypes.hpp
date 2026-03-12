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

    typedef struct {

        int16_t x;
        int16_t y;
        int16_t z;

    } axis3_i16_t;

    class SixAxisStats {

        public:

            Vec3 mean;
            Vec3 variance;

            SixAxisStats() = default;

            SixAxisStats
                (const Vec3& mean, const Vec3& variance) 
                : mean(mean), variance(variance) {}

            SixAxisStats& operator=(const SixAxisStats& other) = default;

     }; // class SixAxisStats


    class EstimatedState { 

        public:

            float dx;      // positive forward
            float dy;      // positive rightward
            float z;       // positive upward
            float dz;      // positive upward
            float phi;     // positive roll right
            float theta;   // positive nose down
            float psi;     // positive nose right

            EstimatedState() = default;

            EstimatedState
                (const float dx,
                 const float dy,
                 const float z,
                 const float dz,
                 const float phi,
                 const float theta,
                 const float psi)
                : 
                    dx(dx),
                    dy(dy),
                    z(z),
                    dz(dz),
                    phi(phi),
                    theta(theta),
                    psi(psi) {}

            EstimatedState& operator=(const EstimatedState& other) = default;
 
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

            Vec3 gyroDps;
            Vec3 accelGs;

            ImuFiltered() = default;

            ImuFiltered(const Vec3 & gyroDps, const Vec3 & accelGs) 
                : gyroDps(gyroDps), accelGs(accelGs) {}

            ImuFiltered& operator=(const ImuFiltered& other) = default;
     };
}
