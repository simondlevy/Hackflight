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

#include <newpidcontrol/althold.hpp>
#include <newpidcontrol/poshold.hpp>
#include <newpidcontrol/stabilize.hpp>

namespace hf {

    class NewPidController {

        public:

            NewPidController() = default;

            NewPidController& operator=(
                    const NewPidController& other) = default;

            static auto RunStabilize(
                    const NewPidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> NewPidController
            {
                const auto stabilize_controller = StabilizeController::Run(
                        pc.stabilize_controller_,
                        airborne,
                        dt,
                        mode,
                        state,
                        setpoint_in);

                return NewPidController(
                        pc.position_controller_,
                        pc.alt_hold_controller_,
                        stabilize_controller,
                        StabilizeController::GetSetpoint(stabilize_controller));
            }

            static auto RunAltHold(
                    const NewPidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> NewPidController
            {
                const auto stabilize_controller = StabilizeController::Run(
                        pc.stabilize_controller_,
                        airborne,
                        dt,
                        mode,
                        state,
                        setpoint_in);

                const auto alt_hold_controller = AltHoldController::Run(
                        pc.alt_hold_controller_,
                        airborne,
                        dt,
                        mode,
                        state,
                        setpoint_in);

                const auto stabilize_setpoint =
                    StabilizeController::GetSetpoint(stabilize_controller);

                const auto alt_hold_setpoint =
                    AltHoldController::GetSetpoint(alt_hold_controller);

                const auto setpoint_out =
                    Setpoint(
                            alt_hold_setpoint.thrust,
                            stabilize_setpoint.roll,
                            stabilize_setpoint.roll,
                            stabilize_setpoint.roll);

                return NewPidController(
                        pc.position_controller_,
                        alt_hold_controller,
                        stabilize_controller,
                        setpoint_out);
            }

            static auto RunPosHold(
                    const NewPidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> NewPidController
            {
                (void)airborne;
                (void)dt;
                (void)mode;
                (void)state;
                (void)setpoint_in;

                return pc;
            }

            static auto GetSetpoint(const NewPidController & pc) -> Setpoint
            {
                return pc.setpoint_;
            }

        private:

            PositionController position_controller_;
            AltHoldController alt_hold_controller_;
            StabilizeController stabilize_controller_;
            Setpoint setpoint_;

            NewPidController(
                    const PositionController & position_controller,
                    const AltHoldController & alt_hold_controller,
                    const StabilizeController & stabilize_controller,
                    const Setpoint & setpoint)
                :
                    position_controller_(position_controller),
                    alt_hold_controller_(alt_hold_controller),
                    stabilize_controller_(stabilize_controller),
                    setpoint_(setpoint) {}
    };
}
