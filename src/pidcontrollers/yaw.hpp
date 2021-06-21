/*
   Angular-velocity-based PID controller for yaw

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "pidcontroller.hpp"
#include "angvel.hpp"

namespace hf {

    class YawPid : public PidController {

        private: 

            // Aribtrary constants
            static constexpr float BIG_YAW_DEMAND = 0.1f;

            // Rate mode uses a rate controller for roll, pitch
            _AngularVelocityPid _yawPid;

        public:

            YawPid(const float Kp, const float Ki) 
            {
                _yawPid.begin(Kp, Ki, 0);
            }

            void modifyDemands(State * state, float * demands)
            {
                demands[DEMANDS_YAW] = _yawPid.compute(demands[DEMANDS_YAW], state->x[State::DPSI]);

                // Prevent "yaw jump" during correction
                demands[DEMANDS_YAW] = rft::Filter::constrainAbs(demands[DEMANDS_YAW], 0.1 + fabs(demands[DEMANDS_YAW]));

                // Reset yaw integral on large yaw command
                if (fabs(demands[DEMANDS_YAW]) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

            virtual void updateReceiver(bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
                _yawPid.updateReceiver(throttleIsDown);
            }

    };  // class RatePid

} // namespace hf
