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

            // Helpers ---------------------------------------------------

            void update(float * demand, float   angvel, float * errI, float * errPrev)
            {
                // Compute err as difference between demand and angular velocity
                float err = *demand - angvel;

                // Compute I term
                *errI = rft::Filter::constrainAbs(*errI + err, _windupMax);

                // Compute D term
                float errD = err - *errPrev;

                *errPrev = err;

                *demand = _Kp * err + _Ki * *errI + _Kd * errD;
            }

        protected:

            virtual void modifyDemands(float * state, float * demands) override
            {
                // Controller state
                static float rollErrI;
                static float rollErrPrev;
                static float pitchErrI;
                static float pitchErrPrev;

                update(&demands[DEMANDS_ROLL], state[State::DPHI],
                        &rollErrI, &rollErrPrev);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward rate (nose-down negative)
                update(&demands[DEMANDS_PITCH], -state[State::DTHETA],
                        &pitchErrI, &pitchErrPrev);
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
