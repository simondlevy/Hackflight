#pragma once

#include <utils.hpp>
#include <pid.hpp>

class YawRateController {

    /*
       Demand is input in degrees per second and output in units appropriate
       for our motors, both nose-right positive.
     */

    public:

        void run(
                const float kp,
                const state_t & state, 
                const float dt, 
                demands_t & demands)
        {
            demands.yaw =
                _pid.run_p(kp, dt, demands.yaw, state.dpsi);
        }

    private:

        PID _pid;
};
