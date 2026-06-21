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

#include <stdio.h>

#include <datatypes.hpp>
#include <mixers/bfquadx.hpp>
#include <sim/dynamics.hpp>

namespace hf {

    class SevJr {

        private:

            // Approximate thrust RPM needed when in perfect hover
            static constexpr float kVehicleHoverRpm = 5850; //35546;

            static constexpr float kThrustScale = 8000;
            static constexpr float kRollPitchYawScale = 1000;

        public:

            static constexpr Dynamics::VehicleParams kVehicleParams = {

                // Actual values
                4.8e-1,  // mass [kg]
                1.7e-1,  // arm length L [m]

                // Reverse-engineered by observation:
                1.0e-6, // thrust coefficient B [F=b*w^2]
                4.1e-7, // I [kg*m^2] for pitch, roll, yaw
                3.9e-11 // drag coefficient D [T=d*w^2] for yaw
            };

            static auto run(const Setpoint & setpoint) -> Setpoint
            {
                // Scale up new setpoint to RPMs
                const Setpoint setpoint_rpms = {
                    kThrustScale * (setpoint.thrust - 0.5f) + kVehicleHoverRpm,
                    kRollPitchYawScale * setpoint.roll,
                    kRollPitchYawScale * setpoint.pitch,
                    kRollPitchYawScale * setpoint.yaw
                };

                static Mixer mixer_;
                mixer_ = hf::Mixer::Run(setpoint_rpms);

                const auto motor_rpms = mixer_.motorvals;

                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const auto o1 = Dynamics::GetOmega2(motor_rpms[0]);
                const auto o2 = Dynamics::GetOmega2(motor_rpms[1]);
                const auto o3 = Dynamics::GetOmega2(motor_rpms[2]);
                const auto o4 = Dynamics::GetOmega2(motor_rpms[3]);

                const auto t = o1 + o2 + o3 + o4;
                const auto r = -o1 - o2 + o3 + o4;
                const auto p = o1 - o2  + o3 - o4;
                const auto y = o1 - o2 - o3 + o4;

                return Setpoint(t, r, p, y);
            }


    }; // class ApexQuad

} // namespace hf
