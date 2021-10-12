/*
   Angular-velocity-based PID controller for yaw

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "../HF_filters.hpp"

#include "../HF_pidcontroller.hpp"

namespace hf {

    class YawPid : public PidController {

        private: 

            // Constants set in constructor ----------------------------

            float _Kp = 0;
            float _Ki = 0;
            float _windupMax = 0;
            float _rateMax = 0;

            virtual void modifyDemands(state_t & state, demands_t & demands, bool ready) override
            {
                static float errorI_;

                // Compute error as difference between yaw demand and angular velocity
                float error = demands.yaw - state.dpsi;

                // Compute I term
                errorI_ = fabs(error) > _rateMax ? 0
                        : ready ? Filter::constrainAbs(errorI_ + error, _windupMax)
                        : errorI_;

                // Adjust yaw demand based on error
                demands.yaw = _Kp * error + _Ki * errorI_;
             }

        public:

            YawPid(const float Kp,
                   const float Ki,
                   const float windupMax=6.0,
                   const float rateMaxDegreesPerSecond=40) 
            {
                _Kp = Kp;
                _Ki = Ki;
                _windupMax = windupMax;
                _rateMax = Filter::deg2rad(rateMaxDegreesPerSecond);
            }

    };  // class YawPid

} // namespace hf
