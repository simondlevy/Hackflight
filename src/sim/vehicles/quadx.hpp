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

    class QuadX {

        public:

            static auto Run(
                    const VehicleParams vparams,
                    const Setpoint & setpoint) -> Setpoint
            {

                // Scale up new setpoint to RPMs
                const Setpoint setpoint_rpms = {
                    Dynamics::GetThrustRpm(vparams, setpoint.thrust),
                    Dynamics::kRollPitchYawScale * setpoint.roll,
                    Dynamics::kRollPitchYawScale * setpoint.pitch,
                    Dynamics::kRollPitchYawScale * setpoint.yaw
                };

                static QuadXMixer mixer_;
                mixer_ = hf::QuadXMixer::Run(setpoint_rpms);

                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const auto o_rr_cw  = Dynamics::RpmToOmegaSquared(mixer_.rr_cw);
                const auto o_rf_ccw = Dynamics::RpmToOmegaSquared(mixer_.rf_ccw);
                const auto o_lr_ccw = Dynamics::RpmToOmegaSquared(mixer_.lr_ccw);
                const auto o_lr_cw  = Dynamics::RpmToOmegaSquared(mixer_.lf_cw);

                return Setpoint(
                         o_rr_cw + o_rf_ccw + o_lr_ccw + o_lr_cw,
                        -o_rr_cw - o_rf_ccw + o_lr_ccw + o_lr_cw,
                         o_rr_cw - o_rf_ccw + o_lr_ccw - o_lr_cw,
                         o_rr_cw - o_rf_ccw - o_lr_ccw + o_lr_cw);
            }

    }; // class QuadX

} // namespace hf
