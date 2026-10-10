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

#include <pidcontrol/pids/altitude.hpp>
#include <pidcontrol/pids/climbrate.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/pids/rollpitch.hpp>
#include <pidcontrol/pids/yaw.hpp>

namespace hf {

    class PidController {

        private:

            static constexpr float kAltitudeMinM = 0.2;
            static constexpr float kAltitudemaxM = 1.0;
            static constexpr float kAltitudeInitM = 0.4;
            static constexpr float kAltitudeIncMps = 0.2;
            static constexpr float kAltitudeLandingM = 0.03;
            static constexpr float kYawMaxDps = 160;     

        public:

            typedef enum {

                kControlStabilize,
                kControlAltHold,
                kControlHover

            } ControlLevel;

            Setpoint setpoint;

            PidController() = default;

            PidController& operator=(
                    const PidController& other) = default;

            PidController(
                    const float altitude_target,
                    const AltitudeController & altitude_pid,
                    const ClimbRateController & climbrate_pid,
                    const RollPitchPid & pitch_pid,
                    const RollPitchPid & roll_pid,
                    const YawPid & yaw_pid,
                    const Setpoint & setpoint)
                :
                    setpoint(setpoint),
                    altitude_target_(altitude_target),
                    altitude_pid_(altitude_pid),
                    climbrate_pid_(climbrate_pid),
                    pitch_pid_(pitch_pid),
                    roll_pid_(roll_pid),
                    yaw_pid_(yaw_pid) {}

            static auto Run(
                    const PidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in,
                    const ControlLevel=kControlHover) -> PidController
            {
                // Altitude hold ---------------------------------------------

                const auto  altitude_target =
                    pc.altitude_target_ == 0 ? kAltitudeInitM :
                    pc.altitude_target_;

                const auto new_altitude_target = Num::ConstrainFloat(
                        altitude_target +
                        setpoint_in.thrust * kAltitudeIncMps * dt,
                        kAltitudeMinM, kAltitudemaxM);

                const auto hovering =
                    mode == kModeHovering || mode == kModeAutonomous;

                const auto altitude_pid =
                    AltitudeController::Run(pc.altitude_pid_, hovering, dt,
                            new_altitude_target, state.z);

                const auto climbrate_pid =
                    ClimbRateController::Run(pc.climbrate_pid_,
                            hovering || (state.z > kAltitudeLandingM),
                            dt,
                            altitude_pid.output, state.dz);

                // Stabilize  ------------------------------------------------

                const auto roll =
                    PositionController::Bypass(setpoint_in.roll);

                const auto pitch =
                    PositionController::Bypass(setpoint_in.pitch);

                const auto roll_pid = RollPitchPid::Run(pc.roll_pid_,
                        dt, airborne, roll, state.phi, state.dphi);

                const auto pitch_pid = RollPitchPid::Run(pc.pitch_pid_,
                        dt, airborne, pitch, state.theta, state.dtheta);

                const auto yaw_pid = YawPid::Run(pc.yaw_pid_,
                        dt, airborne, setpoint_in.yaw * kYawMaxDps, state.dpsi);

                const auto setpoint_out = Setpoint(
                        setpoint_in.thrust,
                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return PidController(
                        new_altitude_target,
                        altitude_pid,
                        climbrate_pid,
                        roll_pid,
                        pitch_pid,
                        yaw_pid,
                        setpoint_out);
            }

        private:

            float altitude_target_;
            AltitudeController altitude_pid_;
            ClimbRateController climbrate_pid_;

            RollPitchPid pitch_pid_;
            RollPitchPid roll_pid_;

            YawPid yaw_pid_;
    };
}
