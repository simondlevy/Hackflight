/*
   PID controller for Level mode

   Copyright (c) 2021 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_state.hpp>
#include <rft_closedloops/pidcontroller.hpp>

#include "demands.hpp"
#include "state.hpp"

namespace hf {

    class LevelPid : public rft::PidController {

        private:

            // Helper class
            class _AnglePid : public rft::Pid {

                private:

                    static constexpr float MAX_ANGLE_DEGREES = 45;

                    // Maximum roll pitch demand is +/-0.5, so to convert demand to 
                    // angle for error computation, we multiply by the folling amount:
                    float _demandMultiplier = 2 * rft::Filter::deg2rad(MAX_ANGLE_DEGREES);

                public:

                    void begin(const float Kp) 
                    {
                        Pid::begin(Kp, 0, 0);
                    }

                    float compute(float demand, float angle)
                    {
                        return Pid::compute(demand*_demandMultiplier, angle);
                    }

            }; // class _AnglePid

            _AnglePid _rollPid;
            _AnglePid _pitchPid;

        public:

            LevelPid(float rollLevelP, float pitchLevelP)
            {
                _rollPid.begin(rollLevelP);
                _pitchPid.begin(pitchLevelP);
            }

            LevelPid(float rollPitchLevelP)
                : LevelPid(rollPitchLevelP, rollPitchLevelP)
            {
            }

            void modifyDemands(rft::State * state, float * demands)
            {
                float * x = ((State *)state)->x;
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL], x[State::STATE_PHI]); 
                demands[DEMANDS_PITCH] = _pitchPid.compute(demands[DEMANDS_PITCH], x[State::STATE_THETA]);
            }

    };  // class LevelPid

} // namespace
