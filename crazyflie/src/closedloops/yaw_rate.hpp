#pragma once

#include <pid.hpp>

class YawRateController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=120,
                const float ki=16.7)
        {
            ClosedLoopController::init(updateRate);

            _pid.init(kp,  ki,  0, 0, _dt, _updateRate, CUTOFF_FREQ, false);

            _pid.setIntegralLimit(INTEGRAL_LIMIT);
        }

        /**
          * Demand is input and output as degrees per second, nose-right
          * positive, and output as arbitrary nose-right positive value to be
          * scaled according to motor characteristics.
          */
         virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            // Yaw demand is nose-right positive, whereas yaw angle psi and its first
            // derivative (angular velocity) dspi are nose-right negative.
            // Hence we negate yaw demand, run the PID closedloop on the
            // negated demand and the angular velocity, and negate the result to get the
            // correct yaw demand.
            const auto raw = -_pid.run(-demands.yaw, state.dpsi); 

            demands.yaw = Num::fconstrain(raw, -OUTPUT_LIMIT, +OUTPUT_LIMIT);
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        static constexpr float CUTOFF_FREQ = 30;
        static constexpr float INTEGRAL_LIMIT = 166.7;
        static constexpr float OUTPUT_LIMIT = INT16_MAX;

        Pid _pid;
};
