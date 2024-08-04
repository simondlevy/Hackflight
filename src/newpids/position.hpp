/*
   Position PID controller for Hackflight

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

    class PositionController {

        /* Demand is input as desired speed in meter per second, output as
           angles in degrees.
         */

        public:

            void run(const state_t & state, demands_t & demands)
            {
                run_axis(demands.roll, state.dy);

                run_axis(demands.pitch, state.dx);
            }

        private:

            static constexpr float KP = 25;

            static void run_axis(float & demand, const float actual)
            {
                demand = KP * (demand - actual);
            }
    };

}
