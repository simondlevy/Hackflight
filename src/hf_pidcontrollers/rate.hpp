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

            void reset(float * errI, float * errD1, float * errD2, float * errPrev)
            {
                *errI = 0;
                *errD1 = 0;
                *errD2 = 0;
                *errPrev = 0;
            }

            void update(
                    float * demand,
                    float   angvel, 
                    float * errI,
                    float * errD1,
                    float * errD2,
                    float * errPrev)
            {
                // Reset integral on quick angular velocity change
                if (fabs(angvel) > _rateMax) {
                    reset(errI, errD1, errD2, errPrev);
                }

                // Compute err as difference between demand and angular velocity
                float err = *demand - angvel;

                // Compute I term
                *errI = rft::Filter::constrainAbs(*errI + err, _windupMax);

                // Compute D term
                float errD = err - *errPrev;

                // Low-pass filter dterm
                *errD2 = *errD1;
                *errD1 = errD;
                *errPrev = err;

                *demand = _Kp * err + _Ki * *errI + _Kd * (errD + *errD1 + *errD2);
            }

        protected:

            virtual void modifyDemands(float * state, float * demands) override
            {
                // Controller state
                static float rollErrI;
                static float rollErrD1;
                static float rollErrD2;
                static float rollErrPrev;
                static float pitchErrI;
                static float pitchErrD1;
                static float pitchErrD2;
                static float pitchErrPrev;

                update(&demands[DEMANDS_ROLL], state[State::DPHI],
                        &rollErrI, &rollErrD1, &rollErrD2, &rollErrPrev);

                // Pitch demand is nose-down positive, so we negate
                // pitch-forward rate (nose-down negative)
                update(&demands[DEMANDS_PITCH], -state[State::DTHETA],
                        &pitchErrI, &pitchErrD1, &pitchErrD2, &pitchErrPrev);
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
