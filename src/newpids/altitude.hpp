/*
   Altitude PID controller for Hackflight

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

    class AltitudeController { 

        /*
           Demand is input as normalized altitude target in meters and output as 
           climb rate in meters-per-second

         */

        public:

            void run(
                    const state_t & state, 
                    const float target,
                    demands_t & demands)
            {

                demands.thrust = KP * (target - state.z);
            }

        private:

            static constexpr float KP = 2.0;
    };

}
