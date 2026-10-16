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
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in) -> AltHoldController
            {
                (void)airborne;
                (void)dt;
                (void)mode;
                (void)state;
                (void)setpoint_in;

                return ac;
            }

            static auto GetSetpoint(const AltHoldController & ac) -> Setpoint
            {
                return ac.setpoint_;
            }

        private:

            Setpoint setpoint_;

    };
}
