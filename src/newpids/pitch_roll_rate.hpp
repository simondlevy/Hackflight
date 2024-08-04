/*
   Pitch and roll anglular velocity PID controller for Hackflight

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

    class PitchRollRateController {

        /*
           Demands are input as angular velocities in degrees per second and
           output in uints appropriate for our motors.
         */

        public:

            void run(
                    const float kp,
                    const state_t & state, 
                    demands_t & demands)
            {
                run_axis(kp, demands.roll, state.dphi);

                run_axis(kp, demands.pitch, state.dtheta);
            }

        private:

            static void run_axis(
                    const float kp,
                    float & demand, 
                    const float actual) 
            {
                demand = kp * (demand - actual);
            }
    };

}
