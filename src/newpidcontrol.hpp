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
                (void)airborne;
                (void)dt;
                (void)mode;
                (void)state;
                (void)setpoint_in;

                return pc;
            }

            static auto RunAltHold(
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

            Setpoint setpoint_;


    };
}
