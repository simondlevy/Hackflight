#pragma once

#include "../closedloop.hpp"
#include "../num.hpp"
#include "../pid.hpp"

class PitchRollRateController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=125,
                const float ki=250,
                const float kd=1.25)
        {
            ClosedLoopController::init(updateRate);

            initPid(kp, ki, kd, _rollPid);

            initPid(kp, ki, kd, _pitchPid);
        }

        /**
          * Demands are input as angular velocities in degrees per second and
          * output as as arbitrary values to be scaled according to motor
          * characteristics:
          *
          * roll:  input roll-right positive => output positive
          *
          * pitch: input nose-up positive => output positive
          */
         virtual void run(
                const state_t & state, demands_t & demands) override
        {
            demands.roll = _rollPid.run(demands.roll, state.dphi);

            demands.pitch = _pitchPid.run(demands.pitch, state.dtheta);

        }

        void resetPids(void)
        {
            _rollPid.reset();

            _pitchPid.reset();
        }

    private:

        static constexpr float INTEGRAL_LIMIT = 33;
        static constexpr float FILTER_CUTOFF = 30;

        Pid _rollPid;
        Pid _pitchPid;

        void initPid(
                const float kp,
                const float ki,
                const float kd,
                Pid  & pid) 
        {
            pid.init(kp,  ki,  kd, 0, _dt, _updateRate, FILTER_CUTOFF, false);

            pid.setIntegralLimit(INTEGRAL_LIMIT);
        }
};
