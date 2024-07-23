/*
  Demand is input as normalized altitude target in meters and output as 
  climb rate in meters-per-second

  */

#pragma once

#include <stdio.h>

#include <utils.hpp>
#include <pid.hpp>

class AltitudeController { 

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
