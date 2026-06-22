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
#include <sim/dynamics.hpp>

namespace hf {

    class SevJr {

        private:

            static constexpr float kThrustScale = 10000;

        public:

            static constexpr Dynamics::VehicleParams kVehicleParams = {

                // Actual values
                4.8e-1,  // mass [kg]
                1.7e-1,  // arm length L [m]

                // Reverse-engineered by observation:
                1.0e-6, // thrust coefficient B [F=b*w^2]
                4.1e-5, // I [kg*m^2] for pitch, roll, yaw
                3.9e-9  // drag coefficient D [T=d*w^2] for yaw
            };

            static auto Run(const Setpoint & setpoint) -> Setpoint
            {
                const auto t_rpm = kThrustScale * setpoint.thrust;
                const auto r_rpm = Dynamics::kRollPitchYawScale * setpoint.roll;
                const auto p_rpm = Dynamics::kRollPitchYawScale * setpoint.pitch;
                const auto y_rpm = Dynamics::kRollPitchYawScale * setpoint.yaw;

                const auto m_se = t_rpm - r_rpm + p_rpm - y_rpm;
                const auto m_ne = t_rpm - r_rpm - p_rpm + y_rpm;
                const auto m_sw = t_rpm + r_rpm + p_rpm + y_rpm;
                const auto m_nw = t_rpm + r_rpm - p_rpm - y_rpm;

                // Equation 6 from Bouabdallah et al 2004 ---------------------

                const auto o_se = Dynamics::RpmToOmegaSquared(m_se);
                const auto o_ne = Dynamics::RpmToOmegaSquared(m_ne);
                const auto o_sw = Dynamics::RpmToOmegaSquared(m_sw);
                const auto o_nw = Dynamics::RpmToOmegaSquared(m_nw);

                const auto t =  o_se + o_ne + o_sw + o_nw;
                const auto r = -o_se - o_ne + o_sw + o_nw;
                const auto p =  o_se - o_ne + o_sw - o_nw;
                const auto y =  o_se - o_ne - o_sw + o_nw;

                return Setpoint(t, r, p, y);
            }


    }; // class ApexQuad

} // namespace hf
