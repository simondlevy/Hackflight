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

                float errI;
                float errD1;
                float errD2;
                float errPrev;

            } controller_state_t;


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

                // Compute err as difference between demand and angular velocity
                float err = demands[demand_axis] - angular_velocity;

                // Compute I term
                controller_state->errI = rft::Filter::constrainAbs(controller_state->errI + err, _windupMax);

                // Compute D term
                float errD = err - controller_state->errPrev;
                float dterm = (controller_state->errD1 + controller_state->errD2 + errD) * _Kd; 
                controller_state->errD2 = controller_state->errD1;
                controller_state->errD1 = errD;
                controller_state->errPrev = err;

                demands[demand_axis] = _Kp * err + _Ki * controller_state->errI + dterm;
            }

        protected:

            virtual void modifyDemands(float * state, float * demands) override
            {
                static controller_state_t _rstate; 
                static controller_state_t _pstate;

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
            }

    };  // class RatePid

} // namespace hf
