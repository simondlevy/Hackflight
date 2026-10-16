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

#include <newpidcontrol/althold/altitude.hpp>
#include <newpidcontrol/althold/climbrate.hpp>

namespace hf {

    class AltHoldController {

        private:

            static constexpr float kMinM = 0.2;
            static constexpr float kmaxM = 1.0;
            static constexpr float kInitM = 0.4;
            static constexpr float kIncMps = 0.2;
            static constexpr float kLandingM = 0.03;

        public:

            AltHoldController() = default;

            AltHoldController& operator=(
                    const AltHoldController& other) = default;

            static auto Run(
                    const AltHoldController & ac,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> AltHoldController
            {
                const auto  altitude_target =
                    ac.altitude_target_ == 0 ? kInitM :
                    ac.altitude_target_;

                const auto new_altitude_target = Num::ConstrainFloat(
                        altitude_target + setpoint_in.thrust * kIncMps * dt,
                        kMinM, kmaxM);

                const auto hovering =
                    mode == kModeHover || mode == kModeAutonomous;

                const auto altitude_pid =
                    AltitudeController::Run(ac.altitude_pid_, hovering, dt,
                            new_altitude_target, state.z);

                const auto climbrate_pid =
                    ClimbRateController::Run(ac.climbrate_pid_,
                            hovering || (state.z > kLandingM),
                            dt,
                            altitude_pid.output, state.dz);

                const auto setpoint_out = 
                    Setpoint(
                            climbrate_pid.output,
                            setpoint_in.roll,
                            setpoint_in.pitch,
                            setpoint_in.roll);

                return AltHoldController(
                        setpoint_out,
                        new_altitude_target,
                        altitude_pid,
                        climbrate_pid);
            }

            static auto GetSetpoint(const AltHoldController & ac) -> Setpoint
            {
                return ac.setpoint_;
            }

        private:

            Setpoint setpoint_;
            float altitude_target_;
            AltitudeController altitude_pid_;
            ClimbRateController climbrate_pid_;

            AltHoldController(
                    const Setpoint & setpoint,
                    const float altitude_target,
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid)
                :
                    setpoint_(setpoint),
                    altitude_target_(altitude_target),
                    altitude_pid_(altitude_pid),
                    climbrate_pid_(climbrate_pid) {}
    };
}
