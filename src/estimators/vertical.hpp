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
                    float & dz_ranger)
            {
                // Rotation matrix adapted from 
                //   https://github.com/bitcraze/crazyflie-firmware/blob/master/src/
                //     modules/src/kalman_core/kalman_core.c#L715
                const auto rz = quat.w * quat.w - quat.x * quat.x -
                    quat.y * quat.y + quat.z*quat.z;

                _integral += ACCEL_SCALE * Utils::GS2MSS * dt * (accel.z - 1) * rz;

                dz_accel = _integral;

                z = h * rz;

                dz_ranger = RANGER_SCALE * (z - _zprev) / dt;

                _zprev = z;
            }

        private:

            static constexpr float ACCEL_SCALE = 3;

            static constexpr float RANGER_SCALE = 0.3125;

            float _integral;

            float _zprev;
    };

}
