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

    class StabilizeController {

        private:

            static constexpr float kYawMaxDps = 160;     

        public:


            StabilizeController() = default;

            StabilizeController& operator=(
                    const StabilizeController& other) = default;

            static auto Run(
                    const StabilizeController & sc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> StabilizeController
            {
                (void)airborne;
                (void)dt;
                (void)mode;
                (void)state;
                (void)setpoint_in;

                return sc;
            }

            static auto GetSetpoint(const StabilizeController & sc) -> Setpoint
            {
                return sc.setpoint_;
            }

        private:

            Setpoint setpoint_;
    };
}
