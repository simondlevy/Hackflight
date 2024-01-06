#pragma once

#include <pid.hpp>
#include <closedloop.hpp>

class AltitudeController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float altitudeKp=2,
                const float altitueKi=0.5,
                const float climbRateKp=25,
                const float climbRateKi=15)
        {
            ClosedLoopController::init(updateRate);

            _altitudePid.init(altitudeKp, altitueKi, 0, 0, _dt, _updateRate,
                    FILTER_CUTOFF, true);

            _altitudePid.setOutputLimit(fmaxf(VEL_MAX, 0.5f)  * VEL_MAX_OVERHEAD);

            _climbRatePid.init(climbRateKp, climbRateKi, 0, 0, _dt, _updateRate,
                    FILTER_CUTOFF, true); 
        }

        /**
         * Demand is input as altitude target in meters and output as 
         * arbitrary positive value to be scaled according to motor
         * characteristics.
         */
        virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            // Set climb rate based on target altitude
            auto climbRate = _altitudePid.run(demands.thrust, state.z);

            // Set thrust for desired climb rate
            demands.thrust = _climbRatePid.run(climbRate, state.dz);
        }

        void resetPids(void)
        {
            _altitudePid.reset();
            _climbRatePid.reset();
        }

        void resetFilters(void)
        {
            _altitudePid.filterReset(_updateRate, FILTER_CUTOFF, true);
            _climbRatePid.filterReset(_updateRate, FILTER_CUTOFF, true);
        }

        void setOutputLimit(const float limit)
        {
            _altitudePid.setOutputLimit(limit);
        }


    private:

        static constexpr float VEL_MAX = 1;
        static constexpr float VEL_MAX_OVERHEAD = 1.10;
        static constexpr float FILTER_CUTOFF = 20;

        Pid _altitudePid;
        Pid _climbRatePid;
};
