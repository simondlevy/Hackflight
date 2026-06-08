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

#include <pidcontrol/pids/rollpitch.hpp>
#include <pidcontrol/pids/yaw.hpp>

namespace hf {

    class StabilizerPidController {

        private:

            static constexpr float YAW_MAX_DPS = 160;     

        public:

            Setpoint setpoint;

            StabilizerPidController() = default;

            StabilizerPidController& operator=(
                    const StabilizerPidController& other) = default;

            StabilizerPidController(
                    const RollPitchPid & pitch_pid,
                    const RollPitchPid & roll_pid,
                    const YawPid & yaw_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                pitch_pid_(pitch_pid),
                roll_pid_(roll_pid),
                yaw_pid_(yaw_pid) {}

            static auto run(
                    const StabilizerPidController & s,
                    const bool airborne,
                    const float dt,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> StabilizerPidController
            {
                const auto roll_pid = RollPitchPid::run(s.roll_pid_,
                        dt, airborne, setpoint_in.roll, state.phi, state.dphi);

                const auto pitch_pid = RollPitchPid::run(s.pitch_pid_,
                        dt, airborne, setpoint_in.pitch, state.theta, state.dtheta);

                const auto yaw_pid = YawPid::run(s.yaw_pid_,
                        dt, airborne, setpoint_in.yaw * YAW_MAX_DPS, state.dpsi);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return StabilizerPidController(
                        roll_pid, pitch_pid, yaw_pid, setpoint_out);
             }

        private:

            RollPitchPid pitch_pid_;
            RollPitchPid roll_pid_;
            YawPid yaw_pid_;
    };
}
