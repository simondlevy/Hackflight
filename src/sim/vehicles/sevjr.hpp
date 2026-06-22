/*
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Gefrral Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Gefrral Public License for more details.
 *
 * You should have received a copy of the GNU Gefrral Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */


#pragma once

#include <datatypes.hpp>
#include <mixers/sevjr.hpp>
#include <sim/dynamics.hpp>

namespace hf {

    class SevJr {

        public:

            static constexpr VehicleParams kVehicleParams = {

                // Actual values
                11.1,    // battery voltage
                1050,    // motor kV
                0.00,    // leg height [m]
                4.8e-1,  // mass [kg]
                1.7e-1,  // arm length L [m]

                // Reverse-engineered
                1.3e-6, // thrust coefficient B [F=b*w^2]
                4.1e-5, // I [kg*m^2] for pitch, roll, yaw
                3.9e-9  // drag coefficient D [T=d*w^2] for yaw
            };

            static auto Run(const Setpoint & setpoint) -> Setpoint
            {
                // Scale up new setpoint to RPMs
                const Setpoint setpoint_rpms = {
                    Dynamics::GetThrustRpm(kVehicleParams, setpoint.thrust),
                    Dynamics::kRollPitchYawScale * setpoint.roll,
                    Dynamics::kRollPitchYawScale * setpoint.pitch,
                    Dynamics::kRollPitchYawScale * setpoint.yaw
                };

                static SevJrMixer mixer_;
                mixer_ = hf::SevJrMixer::Run(setpoint_rpms);

                // See Equation 6 from Bouabdallah et al 2004 -----------------

                const auto o_prop_fl_cw =
                    Dynamics::RpmToOmegaSquared(mixer_.prop_fl_cw);

                const auto o_prop_fr_ccw =
                    Dynamics::RpmToOmegaSquared(mixer_.prop_fr_ccw);

                const auto o_prop_r_cw =
                    Dynamics::RpmToOmegaSquared(mixer_.prop_r_cw);

                const auto out = Setpoint(
                        o_prop_fl_cw + o_prop_fr_ccw + o_prop_r_cw,
                        o_prop_fl_cw - o_prop_fr_ccw,
                        -(o_prop_fl_cw + o_prop_fr_ccw)/2 + o_prop_r_cw,
                        0);

                return out;
            }

    }; // class SevJr

} // namespace hf
