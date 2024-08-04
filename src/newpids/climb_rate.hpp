/*
   Climb-rate PID controller for Hackflight

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

namespace hf {

    class ClimbRateController {

        /*
           Demand is input as climb rate in meters per second and output as positive
           value scaled according to motor characteristics.
         */

        public:

            void run(
                    const state_t & state, 
                    const float tbase,
                    const float tscale,
                    const float tmin,
                    const bool flying,
                    demands_t & demands)
            {
                const auto thrustpid = KP * (demands.thrust - state.dz);

                demands.thrust = flying ? thrustpid * tscale + tbase : tmin;
            }

        private:

            static constexpr float KP = 25;
    };

}
