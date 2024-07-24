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

#include <stdio.h>

#include <utils.hpp>
#include <pid.hpp>

class AltitudeController { 

    /*
       Demand is input as normalized altitude target in meters and output as 
       climb rate in meters-per-second

     */

    public:

        void run(
                const state_t & state, 
                const float dt,
                const float target,
                demands_t & demands)
        {

            demands.thrust = _pid.run_pi(KP, KI, ILIMIT, dt, target, state.z);
        }

    private:

        static constexpr float KP = 2.0;
        static constexpr float KI = 0.5;
        static constexpr float ILIMIT = 5000;

        PID _pid;
};
