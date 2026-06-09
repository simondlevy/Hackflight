/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <num.hpp>

namespace hf {

    class PositionController {

        private:

            static constexpr float kP = 10;
            static constexpr float kI = 1;

            static constexpr float kIntegralLimit = 5000;

        public:

            static constexpr float kMaxDemandDegrees = 30;

            float output;

            PositionController() = default;

            PositionController(const PositionController & a) 
                : output(a.output), integral_(a.integral_) {}

            PositionController(const float output, const float integral)
                : output(output), integral_(integral) {}

            PositionController& operator=(const PositionController&) = default;

            /**
             * Demands is input as meters per second and output as angles in
             * degrees.
             */
            static auto Run(
                    const PositionController & c,
                    const bool airborne,
                    const float dt,
                    const float target,
                    const float actual) -> PositionController
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(c.integral_ + error * dt, kIntegralLimit) :
                    0;

                const auto output = airborne ?
                    Num::ConstrainFloat(kP * error + kI * integral, kMaxDemandDegrees) :
                    0;

                return PositionController(output, integral);
            }

        private:

            float integral_;
    };

} // namspace hf
