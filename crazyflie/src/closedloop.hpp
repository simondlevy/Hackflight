#pragma once

#include "clock.hpp"
#include "types.h"

class ClosedLoopController {

    public:

        /**
          * A closed-loop controller accepts a state vector and a set of
          * demands and modifies the demands.
          */
        virtual void run(
                const state_t & state, demands_t & demands) = 0;

    protected:

        Clock::rate_t _updateRate;

        float _dt;

        void init(const Clock::rate_t updateRate)
        {
            _updateRate = updateRate;

            _dt = 1. / updateRate;
        }
};
