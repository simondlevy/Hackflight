#pragma once

#include <utils.hpp>
#include <pid.hpp>

class ClimbRateController {

    /*
       Demand is input as climb rate in meters per second and output as positive
       value scaled according to motor characteristics.
     */

    public:

        void run(
                const state_t & state, 
                const float dt,
                const float tbase,
                const float tscale,
                const float tmin,
                const bool flying,
                demands_t & demands)
        {
            const auto thrustpid = _pid.run_pi(
                    KP, KI, ILIMIT, dt, demands.thrust, state.dz);

            demands.thrust = flying ? thrustpid * tscale + tbase : tmin;
        }

    private:

        PID _pid;

        static constexpr float KP = 25;
        static constexpr float KI = 15;
        static constexpr float ILIMIT = 5000;
};
