#pragma once

#include <datatypes.h>
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
