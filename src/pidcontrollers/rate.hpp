/*
   Angular-velocity-based PID controller for roll and pitch

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
 */

#pragma once

#include <rft_pid.hpp>

#include "pidcontroller.hpp"

namespace hf {

    class _RatePid : public rft::DofPid {

        private: 

            // Arbitrary constants
            static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 
            static constexpr float WINDUP_MAX = 6.0f;

            uint8_t _state_axis = 0;
            uint8_t _demand_axis = 0;
            int8_t _state_direction = 0;

            // Converted to radians from degrees in constructor for efficiency
            float _bigRate = 0;

        protected:

            _RatePid(const float Kp,
                     const float Ki,
                     const float Kd,
                     uint8_t state_axis,
                     uint8_t demand_axis,
                     uint8_t state_direction=+1) 
                : DofPid(Kp, Ki, Kd, WINDUP_MAX)
            {
                // Convert degree parameters to radians for use later
                _bigRate = rft::Filter::deg2rad(BIG_DEGREES_PER_SECOND);

                _state_axis = state_axis;
                _demand_axis = demand_axis;
                _state_direction = state_direction;
            }

            void compute(State * state, float * demands)
            {
                float angvel = state->x[_state_axis];

                // Reset integral on quick angular velocity change
                if (fabs(angvel) > _bigRate) {
                    DofPid::reset();
                }

                demands[_demand_axis] =
                    DofPid::compute(demands[_demand_axis],
                                    _state_direction * angvel);
            }

    };  // class _RatePid


    class RollRatePid : public PidController, protected _RatePid {

        public:

            RollRatePid(const float Kp, const float Ki, const float Kd) 
                : _RatePid(Kp, Ki, Kd, State::DPHI, DEMANDS_ROLL)
            {
            }

            virtual void modifyDemands(State * state, float * demands) override
            {
                _RatePid::compute(state, demands);
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _RatePid::resetOnInactivity(inactive);
            }

    };  // class RollRatePid


    class PitchRatePid : public PidController, protected _RatePid {

        public:

            PitchRatePid(const float Kp, const float Ki, const float Kd) 
                : _RatePid(Kp, Ki, Kd, State::DTHETA, DEMANDS_PITCH, -1)
                  // Pitch demand is nose-down positive, so we negate
                  // pitch-forward rate (nose-down negative) to reconcile them
            {
            }

            virtual void modifyDemands(State * state, float * demands) override
            {
                _RatePid::compute(state, demands);
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _RatePid::resetOnInactivity(inactive);
            }

    };  // class PitchRatePid

    class YawRatePid : public PidController, protected _RatePid {

        public:

            YawRatePid(const float Kp, const float Ki) 
                : _RatePid(Kp, Ki, 0, State::DPSI, DEMANDS_YAW)
            {
            }

            virtual void modifyDemands(State * state, float * demands) override
            {
                _RatePid::compute(state, demands);
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _RatePid::resetOnInactivity(inactive);
            }

    };  // class YawRatePid

} // namespace hf
