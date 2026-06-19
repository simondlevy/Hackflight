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

        private:

                static constexpr double kB = 1.3e-8;  // thrust coefficient B [F=b*w^2]
                static constexpr double kD = 3.9e-11; // drag coefficient D [T=d*w^2] for yaw

                static constexpr double kRho = 1.225; // air density

        public:

            static auto demix(const float * rpms) -> Setpoint
            {
                // Equation 6 ------------------------------------------------

                const auto o1 = GetOmega2(rpms[0]);
                const auto o2 = GetOmega2(rpms[1]);
                const auto o3 = GetOmega2(rpms[2]);
                const auto o4 = GetOmega2(rpms[3]);

                const auto t = kB * (o1 + o2 + o3 + o4);
                const auto r = -kB * o1 - kB * o2 + kB * o3  + kB * o4;
                const auto p = kB * o1 - kB * o2  + kB * o3 - kB * o4;
                const auto y = kD * o1 - kD * o2 - kD * o3 + kD * o4;

                return Setpoint(t, r, p, y);
            }

        private:

            static auto GetOmega2(const double rpm) -> double
            {
                const auto omega = rpm * 2 * M_PI / 60;

                return kRho * omega * omega;
            }

    };

} // namespace hf
