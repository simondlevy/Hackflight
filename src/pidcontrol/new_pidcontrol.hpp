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

#include <pidcontrol/newpids/altitude.hpp>
#include <pidcontrol/newpids/climbrate.hpp>
#include <pidcontrol/newpids/position.hpp>
#include <pidcontrol/newpids/rollpitch.hpp>
#include <pidcontrol/newpids/yaw.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class PidControl {

        private:

            static constexpr float ALTITUDE_INIT_M = 0.4;
            static constexpr float ALTITUDE_MAX_M = 1.0;
            static constexpr float ALTITUDE_MIN_M = 0.2;
            static constexpr float ALTITUDE_INC_MPS = 0.2;

        public:

            Setpoint setpoint;

            PidControl() = default;

            PidControl& operator=(const PidControl& other) = default;

            PidControl(
                    const float altitude_target,
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid,
                    const PositionController & position_x_pid,
                    const PositionController & position_y_pid,
                    const StabilizerPid & stabilizer_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                _altitude_target(altitude_target),
                _altitude_pid(altitude_pid),
                _climbrate_pid(climbrate_pid),
                _position_x_pid(position_x_pid),
                _position_y_pid(position_y_pid),
                _stabilizer_pid(stabilizer_pid) {}

            static auto run(
                    const PidControl & pid,
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const Setpoint & setpoint_in) -> PidControl
            {
                // Altitude hold ---------------------------------------------

                const auto  altitude_target =
                    pid._altitude_target == 0 ? ALTITUDE_INIT_M :
                    pid._altitude_target;

                const auto new_altitude_target = Num::fconstrain(
                        altitude_target +
                        setpoint_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto altitude_pid = AltitudeController::run(pid._altitude_pid,
                        hovering, dt, new_altitude_target, state.z);

                const auto climbrate_pid = ClimbRateController::run(pid._climbrate_pid,
                        hovering, dt, altitude_pid.output, state.z, state.dz);

                const auto thrust = climbrate_pid.output;

                // Position hold ---------------------------------------------

                const auto airborne = thrust > 0;

                // Rotate world-coordinate velocities into body coordinates
                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::DEG2RAD * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto position_y_pid = PositionController::run(pid._position_y_pid,
                        airborne, dt, setpoint_in.roll, dyb);

                const auto position_x_pid = PositionController::run(pid._position_x_pid,
                        airborne, dt, setpoint_in.pitch, dxb);

                //  Stabilization ---------------------------------------------

                const auto setpoint_mid = Setpoint(thrust,
                        position_y_pid.output, position_x_pid.output,
                        setpoint_in.yaw);

                const auto stabilizer_pid = StabilizerPid::run(
                        pid._stabilizer_pid, airborne, dt, state, setpoint_mid);

                return PidControl(new_altitude_target, altitude_pid,
                        climbrate_pid, position_x_pid, position_y_pid,
                        stabilizer_pid, stabilizer_pid.setpoint);
            }

        private:

            float _altitude_target;

            AltitudeController _altitude_pid;

            ClimbRateController _climbrate_pid;

            PositionController _position_x_pid;
            PositionController _position_y_pid;

            StabilizerPid _stabilizer_pid;
    };
}
