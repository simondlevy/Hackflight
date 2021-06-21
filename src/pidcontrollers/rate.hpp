/*
   Angular-velocity-based PID controller for roll and pitch

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "pidcontroller.hpp"
#include "angvel.hpp"

namespace hf {

    class RatePid : public PidController {

        private: 

            // Rate mode uses a rate controller for roll, pitch
            _AngularVelocityPid _rollPid;
            _AngularVelocityPid _pitchPid;

        public:

            RatePid(const float Kp, const float Ki, const float Kd) 
            {
                _rollPid.begin(Kp, Ki, Kd);
                _pitchPid.begin(Kp, Ki, Kd);
            }

            void modifyDemands(state_t * state, float * demands)
            {
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL],  state->angularVel[0]);
                demands[DEMANDS_PITCH] = _pitchPid.compute(demands[DEMANDS_PITCH], state->angularVel[1]);
            }

            virtual void updateReceiver(bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
                _rollPid.updateReceiver(throttleIsDown);
                _pitchPid.updateReceiver(throttleIsDown);
            }

    };  // class RatePid

} // namespace hf
