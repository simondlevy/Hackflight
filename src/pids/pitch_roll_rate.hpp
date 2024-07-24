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

#include <datatypes.h>
#include <utils.hpp>
#include <pid.hpp>

class PitchRollRateController {

    /*
       Demands are input as angular velocities in degrees per second and
       output in uints appropriate for our motors.
     */

    public:

        void run(
                const float kp,
                const float kd,
                const state_t & state, 
                const float dt, 
                demands_t & demands)
        {
            const auto nothrust = demands.thrust == 0;

            run_axis(kp, kd, _roll, demands.roll, dt, state.dphi, nothrust);

            run_axis(kp, kd, _pitch, demands.pitch, dt, state.dtheta, nothrust);
        }

    private:

        PID _roll;
        PID _pitch;

        static void run_axis(
                const float kp,
                const float kd,
                PID & pid, 
                float & demand, 
                const float dt, 
                const float actual, 
                const float reset)
        {
            demand = pid.run_pd(kp, kd, dt, demand, actual, reset);
        }
};
