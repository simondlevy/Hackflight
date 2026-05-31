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

#include <pidcontrol/pids/altitude.hpp>
#include <pidcontrol/pids/climbrate.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class AltHoldPidControl {

        private:

            static constexpr float ALTITUDE_INIT_M = 0.4;
            static constexpr float ALTITUDE_MAX_M = 1.0;
            static constexpr float ALTITUDE_MIN_M = 0.2;
            static constexpr float ALTITUDE_INC_MPS = 0.2;

        public:

            Setpoint setpoint;

            AltHoldPidControl() = default;

            AltHoldPidControl& operator=(const AltHoldPidControl& other) = default;

            AltHoldPidControl(
                    const float altitude_target,
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid,
                    const StabilizerPid & stabilizer_pid,
                    const Setpoint & setpoint)
                : setpoint(setpoint),
                _altitude_target(altitude_target),
                _altitude_pid(altitude_pid),
                _climbrate_pid(climbrate_pid),
                _stabilizer_pid(stabilizer_pid) {}

            static auto run(
                    const AltHoldPidControl & pid,
                    const float dt,
                    const mode_e mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> AltHoldPidControl
            {
                // Altitude hold ---------------------------------------------

                const auto  altitude_target =
                    pid._altitude_target == 0 ? ALTITUDE_INIT_M :
                    pid._altitude_target;

                const auto new_altitude_target = Num::fconstrain(
                        altitude_target +
                        setpoint_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto hovering =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const auto altitude_pid =
                    AltitudeController::run(pid._altitude_pid, hovering, dt,
                            new_altitude_target, state.z);

                const auto climbrate_pid =
                    ClimbRateController::run(pid._climbrate_pid, hovering, dt,
                            altitude_pid.output, state.z, state.dz);

                const auto thrust = climbrate_pid.output;

                const auto airborne = thrust > 0;

                /*
                static uint32_t _count;
                printf("%f,%f,%f,%f,%f,%f\n",
                        _count * dt,
                        new_altitude_target,
                        altitude_pid.output,
                        climbrate_pid.output,
                        state.z,
                        state.dz
                        );
                _count++;*/

                //  Stabilization ---------------------------------------------

                const auto setpoint_mid = Setpoint(thrust,
                        setpoint_in.roll,
                        setpoint_in.pitch,
                        setpoint_in.yaw);

                const auto stabilizer_pid = StabilizerPid::run(
                        pid._stabilizer_pid, airborne, dt, state, setpoint_mid);

                return AltHoldPidControl(
                        new_altitude_target,
                        altitude_pid,
                        climbrate_pid,
                        stabilizer_pid,
                        stabilizer_pid.setpoint);
            }

        private:

            float _altitude_target;

            AltitudeController _altitude_pid;

            ClimbRateController _climbrate_pid;

            StabilizerPid _stabilizer_pid;
    };
}
