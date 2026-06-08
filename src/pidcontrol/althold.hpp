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
                altitude_target_(altitude_target),
                altitude_pid_(altitude_pid),
                climbrate_pid_(climbrate_pid) {}

            static auto Run(
                    const AltHoldPidController & pid,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> AltHoldPidController
            {
                // Altitude hold ---------------------------------------------

                const auto  altitude_target =
                    pid.altitude_target_ == 0 ? ALTITUDE_INIT_M :
                    pid.altitude_target_;

                const auto newaltitude_target_ = Num::ConstrainFloat(
                        altitude_target +
                        setpoint_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto hovering =
                    mode == MODE_HOVERING || mode == MODE_AUTONOMOUS;

                const auto airborne = hovering || (state.z > ALTITUDE_LANDING_M);

                const auto altitude_pid =
                    AltitudeController::Run(pid.altitude_pid_, hovering, dt,
                            newaltitude_target_, state.z);

                const auto climbrate_pid =
                    ClimbRateController::Run(pid.climbrate_pid_, airborne, dt,
                            altitude_pid.output, state.dz);

                return AltHoldPidController(
                        newaltitude_target_,
                        altitude_pid,
                        climbrate_pid,
                        climbrate_pid.output);
            }

        private:

            float altitude_target_;

            AltitudeController altitude_pid_;

            ClimbRateController climbrate_pid_;
    };
}
