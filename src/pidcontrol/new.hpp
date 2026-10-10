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

#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/pids/rollpitch.hpp>
#include <pidcontrol/pids/yaw.hpp>

namespace hf {

    class PidController {

        private:

            static constexpr float kYawMaxDps = 160;     

        public:

            Setpoint setpoint;

            PidController() = default;

            PidController& operator=(
                    const PidController& other) = default;

            PidController(
                    const RollPitchPid & pitch_pid,
                    const RollPitchPid & roll_pid,
                    const YawPid & yaw_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                pitch_pid_(pitch_pid),
                roll_pid_(roll_pid),
                yaw_pid_(yaw_pid) {}

            static auto Run(
                    const PidController & s,
                    const bool airborne,
                    const float dt,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PidController
            {
                const auto roll =
                    PositionController::Bypass(setpoint_in.roll);

                const auto pitch =
                    PositionController::Bypass(setpoint_in.pitch);

                const auto roll_pid = RollPitchPid::Run(s.roll_pid_,
                        dt, airborne, roll, state.phi, state.dphi);

                const auto pitch_pid = RollPitchPid::Run(s.pitch_pid_,
                        dt, airborne, pitch, state.theta, state.dtheta);

                const auto yaw_pid = YawPid::Run(s.yaw_pid_,
                        dt, airborne, setpoint_in.yaw * kYawMaxDps, state.dpsi);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return PidController(
                        roll_pid, pitch_pid, yaw_pid, setpoint_out);
             }

        private:

            RollPitchPid pitch_pid_;
            RollPitchPid roll_pid_;
            YawPid yaw_pid_;
    };
}
