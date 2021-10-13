/*
   PID controller for Level mode

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "../HF_pidcontroller.hpp"

#include "../HF_utils.hpp"

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

            virtual void modifyDemands(state_t & state, demands_t & demands, bool ready) override
            {
                (void)ready;

                demands.roll = _Kp * (demands.roll * _dmdscale - state.phi);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward (nose-down negative) to reconcile them
                demands.pitch = _Kp * (demands.pitch * _dmdscale + state.theta);
            }

        public:

            LevelPid(const float Kp, const float maxAngleDegrees=45)
            {
                _Kp = Kp;

                _dmdscale = 2 * deg2rad(maxAngleDegrees);
            }

    }; // class LevelPid

} // namespace hf
