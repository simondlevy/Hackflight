/*
   Optical-flow filter

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

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class OpticalFlowFilter {

        public:

            static axis2_t run(
                    const axis2_t flow,
                    const axis3_t gyro,
                    const float h,
                    const float dt)
            {
                (void)gyro;
                (void)h;
                (void)dt;

                /*
                const auto theta = 2 * sin(Utils::DEG2RAD * ANGLE / 2);

                const auto dx = (h * theta * flow.x) / (dt * NPIX) -
                    h * gyro.y;

                const auto dy = (h * theta * flow.y) / (dt * NPIX) -
                    h * gyro.x;
                    */

                const auto dx = flow.x;
                const auto dy = flow.y;

                return axis2_t { dx, dy };
            }

        private:

            // For PMW3901 optiacal flow sensor
            static constexpr float NPIX = 35;

            static constexpr float ANGLE = 42;

    };

}
