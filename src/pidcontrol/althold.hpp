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

namespace hf {

    class AltHoldPidController {

        private:

            static constexpr float ALTITUDE_MIN_M = 0.2;
            static constexpr float ALTITUDE_MAX_M = 1.0;
            static constexpr float ALTITUDE_INIT_M = 0.4;
            static constexpr float ALTITUDE_INC_MPS = 0.2;

        public:

            static constexpr float ALTITUDE_LANDING_M = 0.03;

            float thrust;

            AltHoldPidController() = default;

            AltHoldPidController& operator=(
                    const AltHoldPidController& other) = default;

            AltHoldPidController(
                    const float altitude_target,
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid,
                    const float thrust)
                : thrust(thrust),
                _altitude_target(altitude_target),
                _altitude_pid(altitude_pid),
                _climbrate_pid(climbrate_pid) {}

            static auto run(
                    const AltHoldPidController & pid,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> AltHoldPidController
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

                const auto airborne = hovering || (state.z > ALTITUDE_LANDING_M);

                const auto altitude_pid =
                    AltitudeController::run(pid._altitude_pid, hovering, dt,
                            new_altitude_target, state.z);

                const auto climbrate_pid =
                    ClimbRateController::run(pid._climbrate_pid, airborne, dt,
                            altitude_pid.output, state.dz);

                return AltHoldPidController(
                        new_altitude_target,
                        altitude_pid,
                        climbrate_pid,
                        climbrate_pid.output);
            }

        private:

            float _altitude_target;

            AltitudeController _altitude_pid;

            ClimbRateController _climbrate_pid;
    };
}
