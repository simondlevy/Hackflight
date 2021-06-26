/*
   Angular-velocity-based PID controller for roll and pitch

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "pidcontroller.hpp"

#include <rft_closedloops/pidcontrollers/angvel.hpp>

namespace hf {

    class RatePid : public PidController {

        private: 

            // Rate mode uses a rate controller for roll, pitch
            rft::AngularVelocityPid _rollPid;
            rft::AngularVelocityPid _pitchPid;

        public:

            RatePid(const float Kp, const float Ki, const float Kd) 
            {
                _rollPid.begin(Kp, Ki, Kd);
                _pitchPid.begin(Kp, Ki, Kd);
            }

            virtual void modifyDemands(State * state, float * demands) override
            {
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL],  state->x[State::DPHI]);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward rate (nose-down negative) to reconcile them
                demands[DEMANDS_PITCH] =
                    _pitchPid.compute(demands[DEMANDS_PITCH],
                            -state->x[State::DTHETA]);
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _rollPid.resetOnInactivity(inactive);
                _pitchPid.resetOnInactivity(inactive);
            }

    };  // class RatePid

} // namespace hf
