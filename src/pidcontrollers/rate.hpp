/*
   Rate PID controller (roll, pitch)

   Copyright (c) 2021 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>
#include <RFT_state.hpp>
#include <rft_closedloops/pidcontrollers/angvel.hpp>

#include "demands.hpp"

namespace hf {

    class RatePid : public rft::PidController {

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

            void modifyDemands(rft::State * state, float * demands)
            {
                float * x = ((State *)state)->x;

                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL], x[State::STATE_DPHI]);

                demands[DEMANDS_PITCH] = _pitchPid.compute(demands[DEMANDS_PITCH], x[State::STATE_DTHETA]);
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _rollPid.resetOnInactivity(inactive);
                _pitchPid.resetOnInactivity(inactive);
            }

    };  // class RatePid

} // namespace hf
