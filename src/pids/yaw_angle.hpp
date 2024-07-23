#pragma once

#include <utils.hpp>

#include <pid.hpp>

class YawAngleController {

    /*
       Demand is input as desired angle normalized to [-1,+1] and output
       as degrees per second, both nose-right positive.
     */

    public:

        void run(
                const state_t & state, 
                const float dt, 
                demands_t & demands)
        {
            static float _target;

            const auto target = cap (_target + ANGLE_MAX * demands.yaw * dt);

            demands.yaw = _pid.run_pd(KP, KD, dt, target, state.psi);

            // Reset target on zero thrust
            _target =  demands.thrust == 0 ? state.psi : target;
        }

    private:

        static constexpr float KP = 6;
        static constexpr float KD = 0.25;
        static constexpr float ANGLE_MAX = 200;

        PID _pid;

        static float cap(const float angle)
        {
            const float angle1 = angle > 180 ? angle - 360 : angle;

            return angle1 < -180 ? angle1 + 360 : angle1;
        }
};
