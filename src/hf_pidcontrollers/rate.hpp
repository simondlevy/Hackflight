/*
   Angular-velocity-based PID controller for roll and pitch

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_filters.hpp>

#include "HF_pidcontroller.hpp"

namespace hf {

    class RatePid : public PidController {

        private: 

            // Constants set in constructor ----------------------------

            float _Kp = 0;
            float _Ki = 0;
            float _Kd = 0;
            float _windupMax = 0;
            float _rateMax = 0;

            // Controller state ----------------------------------------

            typedef struct {

                float errorI;
                float deltaError1;
                float deltaError2;
                float errorPrev;

            } controller_state_t;


            controller_state_t _rstate = {};
            controller_state_t _pstate = {};

            // Helpers ---------------------------------------------------

            void reset(controller_state_t * controller_state)
            {
                memset(controller_state, 0, sizeof(controller_state_t));
            }

            void modifyDemand(float angular_velocity,
                              float * demands,
                              uint8_t demand_axis,
                              controller_state_t * controller_state)
            {
                // Reset integral on quick angular velocity change
                if (fabs(angular_velocity) > _rateMax) {
                    reset(controller_state);
                }

                // Compute error as difference between demand and angular velocity
                float error = demands[demand_axis] - angular_velocity;

                // Compute I term
                controller_state->errorI = rft::Filter::constrainAbs(controller_state->errorI + error, _windupMax);

                // Compute D term
                float deltaError = error - controller_state->errorPrev;
                float dterm = (controller_state->deltaError1 + controller_state->deltaError2 + deltaError) * _Kd; 
                controller_state->deltaError2 = controller_state->deltaError1;
                controller_state->deltaError1 = deltaError;
                controller_state->errorPrev = error;

                demands[demand_axis] = _Kp * error + _Ki * controller_state->errorI + dterm;
            }

        protected:

            virtual void modifyDemands(float * state, float * demands) override
            {
                modifyDemand(state[State::DPHI], demands, DEMANDS_ROLL, &_rstate);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward rate (nose-down negative)
                modifyDemand(-state[State::DTHETA], demands, DEMANDS_PITCH, &_pstate);
            }

        public:

            RatePid(const float Kp,
                    const float Ki,
                    const float Kd,
                    const float windupMax=6.0,
                    const float rateMaxDegreesPerSecond=40)
            {
                _Kp = Kp;
                _Ki = Ki;
                _Kd = Kd;
                _windupMax = windupMax;
                _rateMax = rft::Filter::deg2rad(rateMaxDegreesPerSecond);

                reset(&_rstate);
                reset(&_pstate);
            }

    };  // class RatePid

} // namespace hf
