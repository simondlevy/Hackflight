/*
 *  BetaFlight QuadX motor mixer for Hackflight
 *
 *               4:cw   2:ccw
 *                   \ /
 *                    X 
 *                   / \
 *               3:ccw  1:cw
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */


#pragma once

#include <datatypes.hpp>

namespace hf {

    class BFQuadXDemixer {

        public:

            static auto demix(const float * motorvals) -> Setpoint
            {
                static constexpr double kB = 1.3e-8;  // thrust coefficient B [F=b*w^2]
                static constexpr double kD = 3.9e-11; // drag coefficient D [T=d*w^2] for yaw

                static constexpr double kRho = 1.225; // air density

                double omega = 0;
                double omega2 = 0;

                double t = 0;
                double r = 0;
                double p = 0;
                double y = 0;

                // Equation 6 ------------------------------------------------

                omega = motorvals[0] * 2 * M_PI / 60;
                omega2 = kRho * omega * omega; 
                t += kB * omega2;
                r += kB * omega2 * -1;
                p += kB * omega2 * +1;
                y -= kD * omega2 * -1;

                omega = motorvals[1] * 2 * M_PI / 60;
                omega2 = kRho * omega * omega; 
                t += kB * omega2;
                r += kB * omega2 * -1;
                p += kB * omega2 * -1;
                y -= kD * omega2 * +1;

                omega = motorvals[2] * 2 * M_PI / 60;
                omega2 = kRho * omega * omega; 
                t += kB * omega2;
                r += kB * omega2 * +1;
                p += kB * omega2 * +1;
                y -= kD * omega2 * +1;

                omega = motorvals[3] * 2 * M_PI / 60;
                omega2 = kRho * omega * omega; 
                t += kB * omega2;
                r += kB * omega2 * +1;
                p += kB * omega2 * -1;
                y -= kD * omega2 * -1;

                return Setpoint(t, r, p, y);
            }
    };

} // namespace hf
