/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2026 Simon D. Levy
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

#include <pidcontrol/althold.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class HoverPidController {

        public:

            Setpoint setpoint;

            HoverPidController() = default;

            HoverPidController& operator=(
                    const HoverPidController& other) = default;

            HoverPidController(
                    const AltHoldPidController & althold_pid,
                    const PositionController & position_x_pid,
                    const PositionController & position_y_pid,
                    const StabilizerPidController & stabilizer_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                _althold_pid(althold_pid),
                _position_x_pid(position_x_pid),
                _position_y_pid(position_y_pid),
                _stabilizer_pid(stabilizer_pid) {}

            static auto run(
                    const HoverPidController & pid,
                    const float dt,
                    const mode_e mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> HoverPidController
            {
                // Altitude hold ---------------------------------------------

                const auto althold_pid = AltHoldPidController::run(
                        pid._althold_pid, dt, mode, state, setpoint_in);

                const auto airborne = state.z > 0.03; // XXX

                // Position hold ---------------------------------------------

                // Rotate world-coordinate velocities into body coordinates
                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::DEG2RAD * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto position_y_pid =
                    PositionController::run(pid._position_y_pid, airborne, dt,
                            setpoint_in.roll, dyb);

                const auto position_x_pid =
                    PositionController::run(pid._position_x_pid, airborne, dt,
                            setpoint_in.pitch, dxb);

                //  Stabilization ---------------------------------------------

                const auto setpoint_mid = Setpoint(althold_pid.thrust,
                        position_y_pid.output, position_x_pid.output,
                        setpoint_in.yaw);

                const auto stabilizer_pid = StabilizerPidController::run(
                        pid._stabilizer_pid, airborne, dt, state, setpoint_mid);

                return HoverPidController(althold_pid,
                        position_x_pid, position_y_pid,
                        stabilizer_pid, stabilizer_pid.setpoint);
            }

        private:

            AltHoldPidController _althold_pid;

            PositionController _position_x_pid;
            PositionController _position_y_pid;

            StabilizerPidController _stabilizer_pid;
    };
}
