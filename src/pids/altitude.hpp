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

    class AltitudeController {

        public:

            float output;

            AltitudeController() = default;

            AltitudeController(const AltitudeController & other)  = default;

            AltitudeController& operator=(const AltitudeController&) = default;

            /**
             * Demand is input as altitude target in meters and output as 
             * climb rate in meters per second.
             */
            static auto Run(
                    const AltitudeController & controller,
                    const float kP,
                    const float kI,
                    const float kIntegralLimit,
                    const float kClimbRateMax,
                    const float kClimbRateMaxOverhead,
                    const bool airborne,
                    const float dt,
                    const float target,
                    const float actual) -> AltitudeController
            {
                const auto error = target - actual;

                const auto integral = airborne ?
                    Num::ConstrainFloat( controller.integral_ + error * dt,
                            kIntegralLimit) :
                    0;

                const auto output = 
                    Num::ConstrainFloat(kP * error + kI * integral,
                            fmaxf(kClimbRateMax, 0.5f)  * kClimbRateMaxOverhead);

                return AltitudeController(output, integral);
            }

        private:

            float integral_;

            AltitudeController(const float output, const float integral)
                : output(output), integral_(integral) {}
    };
}
