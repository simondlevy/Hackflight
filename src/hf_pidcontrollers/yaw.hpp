/*
   Angular-velocity-based PID controller for yaw

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>

#include "HF_pidcontroller.hpp"

namespace hf {

    class YawPid : public PidController {

        private: 

            // Constants set in constructor ----------------------------

            float _Kp = 0;
            float _Ki = 0;
            float _windupMax = 0;
            float _rateMax = 0;

            // Controller state ----------------------------------------
            float _errorI = 0;

            // Helper
            void reset(void)
            {
                _errorI = 0;
            }

            virtual void modifyDemands(float * state, float * demands) override
            {
                // Compute error as difference between yaw demand and angular velocity
                float error = demands[DEMANDS_YAW] - state[State::DPSI];

                // Reset integral on quick angular velocity change
                if (fabs(error) > _rateMax) {
                    reset();
                }

                // Compute I term
                _errorI = rft::Filter::constrainAbs(_errorI + error, _windupMax);

                // Adjust yaw demand based on error
                demands[DEMANDS_YAW] = _Kp * error + _Ki * _errorI;
             }

            virtual void resetOnInactivity(bool inactive) override
            {
                if (inactive) {
                    reset();
                }
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
                _rateMax = rft::Filter::deg2rad(rateMaxDegreesPerSecond);

                reset();
            }

    };  // class YawPid

} // namespace hf
