/*
   Yaw PID controller based on angular velocity

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>
#include <RFT_state.hpp>
#include <rft_closedloops/pidcontroller.hpp>

#include "demands.hpp"
#include "receiver.hpp"
#include "pidcontrollers/angvel.hpp"

namespace hf {

    class YawPid : public rft::PidController {

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

            void modifyDemands(rft::State * state, float * demands)
            {
                float * x = ((State *)state)->x;

                // Prevent "yaw jump" during correction
                demands[DEMANDS_YAW] =
                    rft::Filter::constrainAbs(demands[DEMANDS_YAW], 0.1 + fabs(demands[DEMANDS_YAW]));

                // Reset yaw integral on large yaw command
                if (fabs(demands[DEMANDS_YAW]) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _yawPid.resetOnInactivity(inactive);
            }

    };  // class RatePid

} // namespace hf
