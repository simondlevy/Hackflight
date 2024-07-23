#pragma once

#include <utils.hpp>
#include <pid.hpp>

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
