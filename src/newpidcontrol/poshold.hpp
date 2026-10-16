/**
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <newpidcontrol/poshold/position.hpp>

namespace hf {

    class PosHoldController {

        public:

            PosHoldController() = default;

            PosHoldController& operator=(
                    const PosHoldController& other) = default;

            static auto Run(
                    const PosHoldController & pc,
                    const bool airborne,
                    const float dt,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PosHoldController
            {

                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::kDeg2Rad * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto position_y_pid =
                    PositionController::Run(pc.position_y_pid_, airborne, dt,
                            setpoint_in.roll, dyb);

                const auto position_x_pid =
                    PositionController::Run(pc.position_x_pid_, airborne, dt,
                            setpoint_in.pitch, dxb);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        position_y_pid.output,
                        position_x_pid.output,
                        setpoint_in.yaw);

                return PosHoldController(
                        setpoint_out,
                        position_x_pid,
                        position_y_pid);
            }            

            static auto GetSetpoint(const PosHoldController & pc) -> Setpoint
            {
                return pc.setpoint_;
            }

        private:

            Setpoint setpoint_;
            PositionController position_x_pid_;
            PositionController position_y_pid_;

            PosHoldController(
                    const Setpoint & setpoint,
                    const PositionController & position_x_pid,
                    const PositionController & position_y_pid)
                :
                    setpoint_(setpoint),
                    position_x_pid_(position_x_pid),
                    position_y_pid_(position_y_pid) {}
    };
}
