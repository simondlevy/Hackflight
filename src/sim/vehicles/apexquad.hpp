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

namespace hf {

    class ApexQuad {

        private:

            static constexpr float kThrustScale = 70000;

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

            static auto Run(const Setpoint & setpoint) -> Setpoint
            {
                // Scale up new setpoint to RPMs
                const Setpoint setpoint_rpms = {
                    kThrustScale * setpoint.thrust,
                    Dynamics::kRollPitchYawScale * setpoint.roll,
                    Dynamics::kRollPitchYawScale * setpoint.pitch,
                    Dynamics::kRollPitchYawScale * setpoint.yaw
                };

                static Mixer mixer_;
                mixer_ = hf::Mixer::Run(setpoint_rpms);

                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const auto o_rr_cw = Dynamics::RpmToOmegaSquared(mixer_.rr_cw);
                const auto o_rf_ccw = Dynamics::RpmToOmegaSquared(mixer_.rf_ccw);
                const auto o_lr_ccw = Dynamics::RpmToOmegaSquared(mixer_.lr_ccw);
                const auto o_lr_cw = Dynamics::RpmToOmegaSquared(mixer_.lf_cw);

                const auto t =  o_rr_cw + o_rf_ccw + o_lr_ccw + o_lr_cw;
                const auto r = -o_rr_cw - o_rf_ccw + o_lr_ccw + o_lr_cw;
                const auto p =  o_rr_cw - o_rf_ccw + o_lr_ccw - o_lr_cw;
                const auto y =  o_rr_cw - o_rf_ccw - o_lr_ccw + o_lr_cw;

                return Setpoint(t, r, p, y);
            }


    }; // class ApexQuad

} // namespace hf
