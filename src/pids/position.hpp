/*
  Position-hold PID controller

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

    /**
        Input is meter/second demands and actual velocities; output is roll/pitch
        angles
     */
    class PositionPid {

        private:

            static constexpr float KP = 10;

        public:

            static void run(const state_t & state, demands_t & demands)
            {
                printf("roll=%+3.3f  dy=%+3.3f\n", demands.roll, state.dy);

                demands.roll = KP * (demands.roll - state.dy);
                demands.pitch = KP * (demands.pitch - state.dx);
            }

    };

}
