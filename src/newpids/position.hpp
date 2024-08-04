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

#include <utils.hpp>
#include <pid.hpp>

namespace hf {

    class PositionController {

        /* Demand is input as desired speed in meter per second, output as
           angles in degrees.
         */

        public:

            void run(
                    const state_t & state, const float dt, demands_t & demands) 
            {
                run_axis(_roll_pid, demands.roll, dt, state.dy);

                run_axis(_pitch_pid, demands.pitch, dt, state.dx);
            }

        private:

            static constexpr float KP = 25;

            PID _roll_pid;

            PID _pitch_pid;

            static void run_axis(
                    PID & pid, float & demand, const float dt, const float actual)
            {
                demand = pid.run_p(KP, dt, demand, actual);
            }
    };

}
