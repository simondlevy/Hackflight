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

    class PitchRollAngleController {

        public:

            /**
             * Demand is input as angles in degrees and output as angular
             * velocities in degrees per second:
             *
             * roll: right-down positive
             *
             * pitch: nose-down positive
             */
            static void run(
                    const bool airborne,
                    const float dt,
                    const float state_phi,
                    const float state_theta,
                    const float demand_roll,
                    const float demand_pitch,
                    float & new_demand_roll,
                    float & new_demand_pitch)
            {
                static float _roll_integral;

                static float _pitch_integral;

                new_demand_roll = runAxis(airborne, dt, demand_roll, state_phi,
                        _roll_integral);

                new_demand_pitch =
                    runAxis(airborne, dt, demand_pitch, state_theta,
                            _pitch_integral);
            }

        private:

            static constexpr float KP = 6;
            static constexpr float KI = 3;
            static constexpr float ILIMIT = 20;

            static float runAxis(
                    const bool airborne,
                    const float dt,
                    const float demand,
                    const float measured,
                    float & integral) 
            {
                const auto error = demand - measured;

                integral = airborne ?
                    Num::fconstrain(integral + error * dt, ILIMIT): 0;

                return airborne ? KP * error + KI * integral : 0; 
            }
    };
}
