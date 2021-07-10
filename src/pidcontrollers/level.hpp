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

    class LevelPid : public PidController {

        private:

            // Helper class
            class _AnglePid : public rft::DofPid {

                private:

                    static constexpr float MAX_ANGLE_DEGREES = 45;

                    // Maximum roll pitch demand is +/-0.5, so to convert demand to 
                    // angle for error computation, we multiply by the folling amount:
                    float _demandMultiplier = 2 * rft::Filter::deg2rad(MAX_ANGLE_DEGREES);

                public:

                    void init(const float Kp) 
                    {
                        rft::DofPid::init(Kp, 0, 0);
                    }

                    float compute(float demand, float angle)
                    {
                        return rft::DofPid::compute(demand*_demandMultiplier, angle);
                    }

            }; // class _AnglePid

            _AnglePid _rollPid;
            _AnglePid _pitchPid;

        public:

            LevelPid(float rollLevelP, float pitchLevelP)
            {
                _rollPid.init(rollLevelP);
                _pitchPid.init(pitchLevelP);
            }

            LevelPid(float rollPitchLevelP)
                : LevelPid(rollPitchLevelP, rollPitchLevelP)
            {
            }

            void modifyDemands(State * state, float * demands) override
            {
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL], state->x[State::PHI]); 

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward (nose-down negative) to reconcile them
                demands[DEMANDS_PITCH] = _pitchPid.compute(demands[DEMANDS_PITCH], -state->x[State::THETA]);
            }

    };  // class LevelPid

} // namespace
