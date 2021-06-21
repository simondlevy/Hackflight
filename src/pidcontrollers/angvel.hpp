/*
   Angular-velocity-based PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include "pidcontroller.hpp"

namespace hf {

    // Helper class for all three axes
    class _AngularVelocityPid : public Pid {

        private: 

            // Arbitrary constants
            static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 
            static constexpr float WINDUP_MAX = 6.0f;

            // Converted to radians from degrees in constructor for efficiency
            float _bigAngularVelocity = 0;

        public:

            void begin(const float Kp, const float Ki, const float Kd) 
            {
                Pid::begin(Kp, Ki, Kd, WINDUP_MAX);

                // Convert degree parameters to radians for use later
                _bigAngularVelocity = Filter::deg2rad(BIG_DEGREES_PER_SECOND);
            }

            float compute(float demand, float angularVelocity)
            {
                // Reset integral on quick angular velocity change
                if (fabs(angularVelocity) > _bigAngularVelocity) {
                    reset();
                }

                return Pid::compute(demand, angularVelocity);
            }

    };  // class _AngularVelocityPid

} // namespace hf
