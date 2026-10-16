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

#include <newpidcontrol/stabilize/rollpitch.hpp>
#include <newpidcontrol/stabilize/yaw.hpp>

namespace hf {

    class StabilizeController {

        private:

            static constexpr float kYawMaxDps = 160;     

        public:


            StabilizeController() = default;

            StabilizeController& operator=(
                    const StabilizeController& other) = default;

            static auto Run(
                    const StabilizeController & sc,
                    const bool airborne,
                    const float dt,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> StabilizeController
            {
                const auto roll_pid = RollPitchPid::Run(sc.roll_pid_,
                        dt, airborne, setpoint_in.roll, state.phi, state.dphi);

                const auto pitch_pid = RollPitchPid::Run(sc.pitch_pid_,
                        dt, airborne, setpoint_in.pitch, state.theta, state.dtheta);

                const auto yaw_pid = YawPid::Run(sc.yaw_pid_,
                        dt, airborne, setpoint_in.yaw * kYawMaxDps, state.dpsi);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return StabilizeController(
                        setpoint_out, roll_pid, pitch_pid, yaw_pid);
            }

            static auto GetSetpoint(const StabilizeController & sc) -> Setpoint
            {
                return sc.setpoint_;
            }

        private:

            Setpoint setpoint_;
            RollPitchPid roll_pid_;
            RollPitchPid pitch_pid_;
            YawPid yaw_pid_;

            StabilizeController(
                    const Setpoint & setpoint,
                    const RollPitchPid roll_pid,
                    const RollPitchPid pitch_pid,
                    const YawPid yaw_pid)
                :
                    setpoint_(setpoint),
                    roll_pid_(roll_pid),
                    pitch_pid_(pitch_pid),
                    yaw_pid_(yaw_pid) {}
    };
}
