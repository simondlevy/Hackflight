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

#include <pidcontrol/newpids/rollpitch.hpp>
#include <pidcontrol/newpids/yaw.hpp>

namespace hf {

    class StabilizerPid {

        private:

            static constexpr float YAW_MAX_DPS = 160;     

        public:

            Setpoint setpoint;

            StabilizerPid() = default;

            StabilizerPid& operator=(const StabilizerPid& other) = default;

            StabilizerPid(
                    const RollPitchPid & pitch_pid,
                    const RollPitchPid & roll_pid,
                    const YawPid & yaw_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                _pitch_pid(pitch_pid),
                _roll_pid(roll_pid),
                _yaw_pid(yaw_pid) {}

            static auto run(
                    const StabilizerPid & s,
                    const bool airborne,
                    const float dt,
                    const vehicleState_t & state,
                    const Setpoint & setpoint_in) -> StabilizerPid
            {
                const auto roll_pid = RollPitchPid::run(s._roll_pid,
                        dt, airborne, setpoint_in.roll, state.phi, state.dphi);

                const auto pitch_pid = RollPitchPid::run(s._pitch_pid,
                        dt, airborne, setpoint_in.pitch, state.theta, state.dtheta);

                const auto yaw_pid = YawPid::run(s._yaw_pid,
                        dt, airborne, setpoint_in.yaw * YAW_MAX_DPS, state.dpsi);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return StabilizerPid(
                        roll_pid, pitch_pid, yaw_pid, setpoint_out);
             }

        private:

            RollPitchPid _pitch_pid;
            RollPitchPid _roll_pid;
            YawPid _yaw_pid;
    };
}
