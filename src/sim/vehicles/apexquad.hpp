/*
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
#include <mixers/bfquadx.hpp>
#include <sim/dynamics.hpp>

namespace hf {

    class ApexQuad {

        private:

            // Approximate thrust RPM needed when in perfect hover
            static constexpr float kVehicleHoverRpm = 35546;

        public:

            static constexpr Dynamics::VehicleParams kVehicleParams = {

                // Actual values
                9.0e-2,  // mass [kg]
                5.0e-2,  // arm length L [m]

                // Reverse-engineered by observation:
                1.3e-8, // thrust coefficient B [F=b*w^2]
                4.1e-7, // I [kg*m^2] for pitch, roll, yaw
                3.9e-11 // drag coefficient D [T=d*w^2] for yaw
            };

            static auto run(const Setpoint & setpoint) -> Setpoint
            {
                // Scale up new setpoint to RPMs
                const Setpoint setpoint_rpms = {
                    8000 * (setpoint.thrust - 0.5f) + kVehicleHoverRpm,
                    1000 * setpoint.roll,
                    1000 * setpoint.pitch,
                    1000 * setpoint.yaw
                };

                const auto motor_rpms = MixBFQuadX(setpoint_rpms);

                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const auto o1 = GetOmega2(motor_rpms[0]);
                const auto o2 = GetOmega2(motor_rpms[1]);
                const auto o3 = GetOmega2(motor_rpms[2]);
                const auto o4 = GetOmega2(motor_rpms[3]);

                const auto t = o1 + o2 + o3 + o4;
                const auto r = -o1 - o2 + o3 + o4;
                const auto p = o1 - o2  + o3 - o4;
                const auto y = o1 - o2 - o3 + o4;

                return Setpoint(t, r, p, y);
            }

        private:

            static auto GetOmega2(const double rpm) -> double
            {
                const auto omega = rpm * 2 * M_PI / 60;

                return omega * omega;
            }

    };

} // namespace hf
