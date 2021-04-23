/*
   Angular-velocity-based PID controller for roll, pitch, yaw

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>

#include "receiver.hpp"
#include "state.hpp"
#include "demands.hpp"
#include "pidcontrollers/angvel.hpp"

namespace hf {

    class YawPid : public rft::PidController {

        private: 

            // Aribtrary constants
            static constexpr float BIG_YAW_DEMAND = 0.1f;

            // Rate mode uses a rate controller for roll, pitch
            AngularVelocityPid _yawPid;

        public:

            YawPid(const float Kp_yaw, const float Ki_yaw) 
            {
                _yawPid.begin(Kp_yaw, Ki_yaw, 0);
            }

            void modifyDemands(rft::State * state, float * demands)
            {
                State * hfstate = (State *)state;

                demands[DEMANDS_YAW] = _yawPid.compute(demands[DEMANDS_YAW], hfstate->x[State::DPSI]);

                // Prevent "yaw jump" during correction
                demands[DEMANDS_YAW] = rft::Filter::constrainAbs(demands[DEMANDS_YAW], 0.1 + fabs(demands[DEMANDS_YAW]));

                // Reset yaw integral on large yaw command
                if (fabs(demands[DEMANDS_YAW]) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

            /* XXX should be replaced by resetOnInactivity()
            virtual void updateReceiver(bool throttleIsDown) override
            {
                // Check throttle-down for integral reset
                _yawPid.updateReceiver(throttleIsDown);
            }*/

    };  // class YawPid

} // namespace hf
