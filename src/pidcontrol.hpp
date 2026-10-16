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

#include <pidcontrol/althold.hpp>
#include <pidcontrol/poshold.hpp>
#include <pidcontrol/stabilize.hpp>

namespace hf {

    class PidController {

        public:

            PidController() = default;

            PidController& operator=(
                    const PidController& other) = default;

            static auto RunStabilize(
                    const PidController & pc,
                    const bool airborne,
                    const float dt,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PidController
            {
                const auto stabilize_controller = StabilizeController::Run(
                        pc.stabilize_controller_, airborne, dt, state,
                        setpoint_in);

                const auto setpoint_stable =
                    StabilizeController::GetSetpoint(stabilize_controller);

                const auto setpoint_out = Setpoint(
                        (setpoint_in.thrust + 1) / 2, // [-1,+1] => [0,1]
                        setpoint_stable.roll,
                        setpoint_stable.pitch,
                        setpoint_stable.yaw);

                return PidController(
                        pc.pos_hold_controller_,
                        pc.alt_hold_controller_,
                        stabilize_controller,
                        setpoint_out);
            }

            static auto RunAltHold(
                    const PidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PidController
            {
                const auto stabilize_controller = StabilizeController::Run(
                        pc.stabilize_controller_, airborne, dt, state,
                        setpoint_in);

                const auto alt_hold_controller = AltHoldController::Run(
                        pc.alt_hold_controller_, dt, mode, state, setpoint_in);

                const auto stabilize_setpoint =
                    StabilizeController::GetSetpoint(stabilize_controller);

                const auto alt_hold_setpoint =
                    AltHoldController::GetSetpoint(alt_hold_controller);

                const auto setpoint_out =
                    Setpoint(
                            alt_hold_setpoint.thrust,
                            stabilize_setpoint.roll,
                            stabilize_setpoint.pitch,
                            stabilize_setpoint.yaw);

                return PidController(
                        pc.pos_hold_controller_,
                        alt_hold_controller,
                        stabilize_controller,
                        setpoint_out);
            }

            static auto RunPosHold(
                    const PidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PidController
            {
                const auto pos_hold_controller = PosHoldController::Run(
                        pc.pos_hold_controller_, airborne, dt, state,
                        setpoint_in);

                const auto pos_hold_setpoint_out =
                    PosHoldController::GetSetpoint(pos_hold_controller);

                const auto stabilize_controller = StabilizeController::Run(
                        pc.stabilize_controller_, airborne, dt, state,
                        pos_hold_setpoint_out);

                const auto alt_hold_controller = AltHoldController::Run(
                        pc.alt_hold_controller_, dt, mode, state,
                        setpoint_in);

                const auto stabilize_setpoint =
                    StabilizeController::GetSetpoint(stabilize_controller);

                const auto alt_hold_setpoint =
                    AltHoldController::GetSetpoint(alt_hold_controller);

                const auto setpoint_out =
                    Setpoint(
                            alt_hold_setpoint.thrust,
                            stabilize_setpoint.roll,
                            stabilize_setpoint.pitch,
                            stabilize_setpoint.yaw);

                return PidController(
                        pos_hold_controller,
                        alt_hold_controller,
                        stabilize_controller,
                        setpoint_out);
            }

            static auto GetSetpoint(const PidController & pc) -> Setpoint
            {
                return pc.setpoint_;
            }

        private:

            PosHoldController pos_hold_controller_;
            AltHoldController alt_hold_controller_;
            StabilizeController stabilize_controller_;
            Setpoint setpoint_;

            PidController(
                    const PosHoldController & pos_hold_controller,
                    const AltHoldController & alt_hold_controller,
                    const StabilizeController & stabilize_controller,
                    const Setpoint & setpoint)
                :
                    pos_hold_controller_(pos_hold_controller),
                    alt_hold_controller_(alt_hold_controller),
                    stabilize_controller_(stabilize_controller),
                    setpoint_(setpoint) {}
    };
}
