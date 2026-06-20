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
#include <pidcontrol/pids/altitude.hpp>
#include <pidcontrol/pids/climbrate.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class HoverPidController {

        private:

            static constexpr float kAltitudeMinM = 0.2;
            static constexpr float kAltitudemaxM = 1.0;
            static constexpr float kAltitudeInitM = 0.4;
            static constexpr float kAltitudeIncMps = 0.2;
            static constexpr float kAltitudeLandingM = 0.03;

        public:

            Setpoint setpoint;

            HoverPidController() = default;

            HoverPidController& operator=(
                    const HoverPidController& other) = default;

            HoverPidController(
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid,
                    const AltHoldPidController & althold_pid,
                    const PositionController & position_x_pid,
                    const PositionController & position_y_pid,
                    const StabilizerPidController & stabilizer_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                alt_hold_pid_(althold_pid),
                altitude_pid_(altitude_pid),
                climbrate_pid_(climbrate_pid),
                position_x_pid_(position_x_pid),
                position_y_pid_(position_y_pid),
                stabilizer_pid_(stabilizer_pid) {}

            static auto Run(
                    const HoverPidController & pid,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> HoverPidController
            {
                // Altitude hold ---------------------------------------------

                const auto althold_pid = AltHoldPidController::Run(
                        pid.alt_hold_pid_, dt, mode, state, setpoint_in);

                const auto airborne = state.z > kAltitudeLandingM;

                // Position hold ---------------------------------------------

                // Rotate world-coordinate velocities into body coordinates
                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::kDeg2Rad * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto position_y_pid =
                    PositionController::Run(pid.position_y_pid_, airborne, dt,
                            setpoint_in.roll, dyb);

                const auto position_x_pid =
                    PositionController::Run(pid.position_x_pid_, airborne, dt,
                            setpoint_in.pitch, dxb);

                //  Stabilization ---------------------------------------------

                const auto setpoint_mid = Setpoint(althold_pid.thrust,
                        position_y_pid.output, position_x_pid.output,
                        setpoint_in.yaw);

                const auto stabilizer_pid = StabilizerPidController::Run(
                        pid.stabilizer_pid_, airborne, dt, state, setpoint_mid);

                return HoverPidController(
                        pid.altitude_pid_,
                        pid.climbrate_pid_,
                        althold_pid,
                        position_x_pid,
                        position_y_pid,
                        stabilizer_pid,
                        stabilizer_pid.setpoint);
            }

        private:

            AltHoldPidController alt_hold_pid_;

            AltitudeController altitude_pid_;
            ClimbRateController climbrate_pid_;

            PositionController position_x_pid_;
            PositionController position_y_pid_;

            StabilizerPidController stabilizer_pid_;
    };
}
