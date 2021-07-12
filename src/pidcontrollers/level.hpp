/*
   PID controller for Level mode

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "pidcontroller.hpp"

#include <rft_closedloops/dofpid.hpp>
#include <RFT_filters.hpp>

namespace hf {

    // Helper class
    class _LevelPid {

        private:

            static constexpr float MAX_ANGLE_DEGREES = 45;

            uint8_t _state_axis = 0;
            uint8_t _demand_axis = 0;
            int8_t _state_direction = 0;

            float _Kp = 0;

            // Maximum roll pitch demand is +/-0.5, so to convert
            // demand to angle for error computation, we multiply by
            // the folling amount.
            float _dmdscale = 2 * rft::Filter::deg2rad(MAX_ANGLE_DEGREES);

        protected:

            _LevelPid(const float Kp,
                     const uint8_t state_axis,
                     const uint8_t demand_axis,
                     int8_t state_direction) 
            {
                _Kp = Kp;
                _state_axis = state_axis;
                _demand_axis = demand_axis;
                _state_direction = state_direction;
            }

            void compute(State * state, float * demands)
            {
                demands[_demand_axis] =
                    _Kp * (demands[_demand_axis] * _dmdscale -
                          _state_direction * state->x[_state_axis]);
            }

    }; // class _LevelPid

    class RollLevelPid : public PidController, protected _LevelPid {

        public:

            RollLevelPid(const float Kp)
                : _LevelPid(Kp, State::PHI, DEMANDS_ROLL, +1)
            {
            }

            void modifyDemands(State * state, float * demands) override
            {
                _LevelPid::compute(state, demands);
            }

    };  // class RollLevelPid

    class PitchLevelPid : public PidController, protected _LevelPid {

        public:

            PitchLevelPid(const float Kp) 
                : _LevelPid(Kp, State::THETA, DEMANDS_PITCH, -1)
                // Pitch demand is nose-down positive, so we negate
                // pitch-forward (nose-down negative) to reconcile them
            {
            }

            void modifyDemands(State * state, float * demands) override
            {
                _LevelPid::compute(state, demands);
            }

    };  // class PitchLevelPid

} // namespace
