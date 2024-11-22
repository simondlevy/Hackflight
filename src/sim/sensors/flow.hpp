/*
 *   Simulated optical flow sensor
 *
 *
 *   For and equation see
 *
 *     https://www.bitcraze.io/documentation/repository/crazyflie-firmware/
 *      master/images/flowdeck_velocity.png
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
#include <utils.hpp>

namespace hf {

    class OpticalFlow {

        public:

            static axis2_t read(
                    const Dynamics & d,
                    const float h,
                    const float dt)
            {
                const auto theta = 2 * sin(Utils::DEG2RAD * FIELD_OF_VIEW / 2);

                const auto flow_dx =
                    dt * NPIX * (h * d._x10 + dxy_true.x) / (h * theta);

                const auto flow_dy =
                    dt * NPIX * (h * d._x8 + dxy_true.y) / (h * theta);

                return axis2_t {flow_dx, flow_dy};
            }

        private:

            // https://wiki.bitcraze.io/_media/projects:crazyflie2:
            //    expansionboards:pot0189-pmw3901mb-txqt-ds-r1.40-280119.pdf
            static constexpr float FIELD_OF_VIEW = 42;

            // https://github.com/bitcraze/Bitcraze_PMW3901
            static constexpr float NPIX = 35;
    };

}
