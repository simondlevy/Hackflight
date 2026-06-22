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
#include <mixers/quadx.hpp>
#include <sim/dynamics.hpp>
#include <sim/vehicles/quadx.hpp>

namespace hf {

    class ApexQuad {

        public:

            static constexpr Dynamics::VehicleParams kVehicleParams = {

                // Actual values
                7.4,     // battery voltage
                7000,    // motor kV
                9.0e-2,  // mass [kg]
                5.0e-2,  // arm length L [m]

                // Reverse-engineered
                1.35 * 1.3e-8, // thrust coefficient B [F=b*w^2]
                4.1e-7, // I [kg*m^2] for pitch, roll, yaw
                3.9e-11 // drag coefficient D [T=d*w^2] for yaw
            };

            static auto Run(const Setpoint & setpoint) -> Setpoint
            {
                return QuadX::Run(kVehicleParams, setpoint);

            }

    }; // class ApexQuad

} // namespace hf
