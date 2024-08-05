/*
   Pitch and roll angle PID controller for Hackflight

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

    class PitchRollController {

        /*
           Demand is input as angles in degrees and output in uints appropriate
           for our motors.  in degrees per second; roll-right / pitch-forward
           positive.
         */

        public:

            static void run(
                    const float k1,
                    const float k2,
                    const state_t & state, 
                    demands_t & demands)
            {
                // Angles => angular rates
                run_axis(k1, demands.roll, state.phi);
                run_axis(k1, demands.pitch, state.theta);

                // Angular rates => units appropriate to our motors
                run_axis(k2, demands.roll, state.dphi);
                run_axis(k2, demands.pitch, state.dtheta);
            }

        private:

            static void run_axis(
                    const float k,
                    float & demand, 
                    const float actual)
            {
                demand = k * (demand - actual);
            }
    };

}
