/*
   Complementary filter for altitude/climb-rate sensor fusion

   Copyright (C) 2024 Simon D. Levy

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

#include <stdio.h>

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class ComplementaryVertical {

        public:

            void getValues(
                    const float dt,
                    const axis3_t & accel,
                    const quaternion_t & quat,
                    const float h,
                    float & z,
                    float & dz_accel,
                    float & dz_ranger,
                    float & dz)
            {
                // Rotation matrix adapted from 
                //   https://github.com/bitcraze/crazyflie-firmware/blob/master/src/
                //     modules/src/kalman_core/kalman_core.c#L715
                const auto rz = quat.w * quat.w - quat.x * quat.x -
                    quat.y * quat.y + quat.z*quat.z;

                _integral += Utils::GS2MSS * dt * (accel.z - 1) * rz;

                dz_accel = _integral;

                z = h * rz;

                dz_ranger = (z - _zprev) / dt;

                dz = (K1 * _integral + K2 * dz_ranger) / 2;

                _zprev = z;
            }

        private:

            static constexpr float K1 = 3;

            static constexpr float K2 = 0.31;

            float _integral;

            float _zprev;
    };

}
