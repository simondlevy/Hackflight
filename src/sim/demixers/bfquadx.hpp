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

    static constexpr int8_t roll[4]  = {-1, -1, +1, +1};
    static constexpr int8_t pitch[4] = {+1, -1, +1, -1};
    static constexpr int8_t yaw[4]   = {-1, +1, +1, -1};

    class Demixer {

        public:

            uint8_t motorcount = 4;

    };

    auto DemixBFQuadX(const float * motorvals) -> Setpoint
    {
        (void)motorvals;

        static constexpr double kB = 1.3e-8;  // thrust coefficient B [F=b*w^2]
        static constexpr double kD = 3.9e-11; // drag coefficient D [T=d*w^2] for yaw

        static constexpr double kRho = 1.225; // air density

        double u1 = 0;
        double u2 = 0;
        double u3 = 0;
        double u4 = 0;

        double omega = 0;
        double omega2 = 0;

        omega = motorvals[0] * 2 * M_PI / 60;
        omega2 = kRho * omega * omega; 
        u1 += kB * omega2;
        u2 += kB * omega2 * -1;
        u3 += kB * omega2 * +1;
        u4 -= kD * omega2 * -1;

        omega = motorvals[1] * 2 * M_PI / 60;
        omega2 = kRho * omega * omega; 
        u1 += kB * omega2;
        u2 += kB * omega2 * -1;
        u3 += kB * omega2 * -1;
        u4 -= kD * omega2 * +1;

        omega = motorvals[2] * 2 * M_PI / 60;
        omega2 = kRho * omega * omega; 
        u1 += kB * omega2;
        u2 += kB * omega2 * +1;
        u3 += kB * omega2 * +1;
        u4 -= kD * omega2 * +1;

        omega = motorvals[3] * 2 * M_PI / 60;
        omega2 = kRho * omega * omega; 
        u1 += kB * omega2;
        u2 += kB * omega2 * +1;
        u3 += kB * omega2 * -1;
        u4 -= kD * omega2 * -1;

        /*
        for (int i=0; i<4; ++i) {

            // RPM => rad/sec
            const auto omega = motorvals[i] * 2 * M_PI / 60;

            // Thrust is squared rad/sec scaled by air density
            const auto omega2 = kRho * omega * omega; 

            // Multiply by thrust coefficient
            u1 += kB * omega2;                  
            u2 += kB * omega2 * roll[i];
            u3 += kB * omega2 * pitch[i];
            u4 += kD * omega2 * -yaw[i];
        }*/


        return Setpoint(u1, u2, u3, u4);
    }

} // namespace hf
