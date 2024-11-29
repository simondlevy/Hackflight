/*
 *   Simulated optical flow sensor
 *
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
#include <sim/sensors/rangefinder.hpp>
#include <utils.hpp>

namespace hf {

    class OpticalFlow {

        public:

            static axis2_t read(const Dynamics & d, const float dt) 
            {
                // Rotate inertial-frame horizontal velocity into body frame
                const auto dx = d.x2 * cos(d.x11) - d.x4 * sin(d.x11);
                const auto dy = d.x2 * sin(d.x11) + d.x4 * cos(d.x11);

                const auto z = d.x5;

                const auto flow_dx = convert(dx, z, dt);

                const auto flow_dy = convert(dy, z, dt);

                return axis2_t {flow_dx, flow_dy};
            }

        private:

            // See ekf.hpp for these constants
            static constexpr float NPIX = 35;
            static constexpr float FIELD_OF_VIEW = 42;
            static constexpr float RESOLUTION = 0.1;

            static float thetapix()
            {
                return 2 * sin(FIELD_OF_VIEW / 2 / Utils::RAD2DEG);
            }

            static float convert(
                    const float d, const float z, const float dt)
            {
                return z == 0 ?  0 :  // Avoid division by zero

                    // This formula inverts the one in
                    //   https://www.bitcraze.io/documentation/repository/
                    //   crazyflie-firmware/master/images/flowdeck_velocity.png
                    (d / z) * (dt * NPIX / thetapix()) / RESOLUTION;
            }
    };

}
