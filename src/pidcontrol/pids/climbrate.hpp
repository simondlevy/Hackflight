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

        public:

            static constexpr float ALTITUDE_LANDING_M = 0.03;

        private:

            static constexpr float KP = 25;
            static constexpr float KI = 15;
            static constexpr float ILIMIT = 5000;

        public:

            float output;

            ClimbRateController() = default;

            ClimbRateController(const ClimbRateController & a) 
                : output(a.output), _integral(a._integral) {}

            ClimbRateController(const float output, const float integral)
                : output(output), _integral(integral) {}

            ClimbRateController& operator=(const ClimbRateController&) = default;

            /**
             * Demand is input as climbrate target in meters per second and
             * output as arbitrary positive value to be scaled according to
             * vehicle characteristics.
             */
            static auto run(
                    const ClimbRateController & controller,
                    const bool hovering,
                    const float dt,
                    const float target,
                    const float z,
                    const float dz) -> ClimbRateController
            {
                const auto airborne = hovering || (z > ALTITUDE_LANDING_M);

                const auto error = target - dz;

                const auto integral = airborne ? 
                    Num::fconstrain(controller._integral + error * dt, ILIMIT) : 0;

                const auto thrust = KP * error + KI * integral;

                const auto output = airborne ? thrust : 0;

                return ClimbRateController(output, integral);
            }

        private:

            float _integral;
    };
}
