#pragma once

#include <pid.hpp>

class YawAngleController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=6,
                const float ki=1,
                const float kd=0.35)
        {
            ClosedLoopController::init(updateRate);

            _pid.init(kp,  ki,  kd, 0, _dt, _updateRate, CUTOFF_FREQ, false);

            _pid.setIntegralLimit(INTEGRAL_LIMIT);
        }

        /**
          * Demand is input as angle in degrees and output as degrees per
          * second, both nose-right positive.
          */
         virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            static float _angleTarget;

            // Yaw angle psi is positive nose-left, whereas yaw demand is
            // positive nose-right.  Hence we negate the yaw demand to
            // accumulate the angle target.
            _angleTarget = cap(_angleTarget - demands.yaw * _dt);

            const auto angleError = cap(_angleTarget - state.psi);

            _pid.setError(angleError);

            // Return the result negated, so demand will still be nose-right
            // positive
            demands.yaw = -_pid.run();

            if (demands.thrust == 0) {

                // Reset the calculated YAW angle for rate control
                _angleTarget = state.psi;
            }
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        static constexpr float CUTOFF_FREQ = 30;
        static constexpr float INTEGRAL_LIMIT = 360;

        static float cap(float angle) 
        {
            float result = angle;

            while (result > 180.0f) {
                result -= 360.0f;
            }

            while (result < -180.0f) {
                result += 360.0f;
            }

            return result;
        }

        Pid _pid;
};
