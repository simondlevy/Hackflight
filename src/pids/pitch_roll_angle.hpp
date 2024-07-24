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

#include <utils.hpp>
#include <pid.hpp>

class PitchRollAngleController {

    /*
       Demand is input as angles in degrees and output as angular velocities
       in degrees per second; roll-right / pitch-forward positive.
     */

    public:

        void run(
                const float kp,
                const state_t & state, 
                const float dt, 
                demands_t & demands)
        {
            run_axis(kp, _roll_pid, demands.roll, dt, state.phi);

            run_axis(kp, _pitch_pid, demands.pitch, dt, state.theta);
        }

    private:

        PID _roll_pid;
        PID _pitch_pid;

        static void run_axis(
                const float kp,
                PID & pid, 
                float & demand, 
                const float dt, 
                const float actual) 
        {
            demand = pid.run_p(kp, dt, demand, actual);
        }
};
