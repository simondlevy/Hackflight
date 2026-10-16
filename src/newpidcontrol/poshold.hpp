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

namespace hf {

    class PosHoldController {

        public:

            PosHoldController() = default;

            PosHoldController& operator=(
                    const PosHoldController& other) = default;

            static auto Run(
                    const PosHoldController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> PosHoldController
            {
                (void)airborne;
                (void)dt;
                (void)mode;
                (void)state;
                (void)setpoint_in;

                return pc;
            }            

            static auto GetSetpoint(const PosHoldController & pc) -> Setpoint
            {
                return pc.setpoint_;
            }

        private:

            Setpoint setpoint_;
    };
}
