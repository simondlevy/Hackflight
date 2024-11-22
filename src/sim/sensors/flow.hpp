/*
 *   Optical flow simulator
 *
 *   Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, in version 3.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.hpp>

namespace hf {

    class OpticalFlow {

        static axis2_t read(
                const axis2_t dxy_true,
                const axis3_t gyro,
                const float h,
                const float dt)
        {
            const auto theta = 2 * sin(Utils::DEG2RAD * FLOW_ANGLE / 2);

            const auto flow_dx =
                dt * FLOW_NPIX * (h * gyro.y + dxy_true.x) / (h * theta);

            const auto flow_dy =
                dt * FLOW_NPIX * (h * gyro.x + dxy_true.y) / (h * theta);

            return axis2_t {flow_dx, flow_dy};
        }

    };

}
