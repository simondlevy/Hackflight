/*
   PID controller for Level mode

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pidcontroller.hpp"

#include <RFT_filters.hpp>

namespace hf {

    class LevelPid : public PidController {

        private:

            // Constants set in constructor ----------------------------

            float _Kp = 0;

            // Maximum roll pitch demand is +/-0.5, so to convert
            // demand to angle for error computation, we multiply by
            // the following amount.

            float _dmdscale = 0;

            // ----------------------------------------------------------

        protected:

            virtual void modifyDemands(float * state, float * demands) override
            {
                demands[DEMANDS_ROLL] = 
                    _Kp * (demands[DEMANDS_ROLL] * _dmdscale 
                            - state[State::PHI]);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward (nose-down negative) to reconcile them
                demands[DEMANDS_PITCH] = 
                    _Kp * (demands[DEMANDS_PITCH] * _dmdscale 
                            + state[State::THETA]);
            }

        public:

            LevelPid(const float Kp, const float maxAngleDegrees=45)
            {
                _Kp = Kp;

                _dmdscale = 2 * rft::Filter::deg2rad(maxAngleDegrees);
            }

    }; // class LevelPid

} // namespace hf
