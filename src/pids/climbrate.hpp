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

    class ClimbRateController {

        private:

            static constexpr float kP = 1;
            static constexpr float kI = 0;

            static constexpr float kIntegralLimit = 0;

        public:

            float output;

            ClimbRateController() = default;

            ClimbRateController(const ClimbRateController & a) 
                : output(a.output), integral_(a.integral_) {}

            ClimbRateController(const float output, const float integral)
                : output(output), integral_(integral) {}

            ClimbRateController& operator=(const ClimbRateController&) = default;

            /**
             * Demand is input as climbrate target in meters per second and
             * output so that neutral (hover) = 0.5.
             */
            static auto Run(
                    const ClimbRateController & controller,
                    const bool airborne,
                    const float dt,
                    const float target,
                    const float dz) -> ClimbRateController
            {
                const auto error = target - dz;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(controller.integral_ + error * dt,
                            kIntegralLimit) : 0;

                const auto thrust = kP * error + kI * integral;

                const auto output = airborne ?
                    Num::ConstrainFloat(0.5 + thrust, 0, 1) :
                    0;

                return ClimbRateController(output, integral);
            }

        private:

            float integral_;
    };
}
