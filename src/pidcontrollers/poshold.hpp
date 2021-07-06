/*
   Position-hold PID controller

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include "pidcontroller.hpp"

#include <rft_closedloops/dofpid.hpp>

namespace hf {

    class PositionHoldPid : public PidController {

        private: 

            // Arbitrary constants: for details see http://ardupilot.org/copter/docs/loiter-mode.html
            static constexpr float PILOT_VELXY_MAX  = 5.0f;
            static constexpr float STICK_DEADBAND = 0.20;   

            // P controller for position.  This will serve as the set-point for velocity PID.
            rft::DofPid _posPid;

            // PID controller for velocity
            rft::DofPid _velPid;

            // Will be reset each time we re-enter deadband.
            float __yTarget = 0;

            // Tracks whether we just entered deadband
            bool _inBandPrev = false;

        protected:

            void modifyDemands(State * state, float * demands) override
            {
                bool didReset = false;
                float y = state->x[State::Y];

                // Is stick demand in deadband?
                bool inBand = fabs(demands[DEMANDS_ROLL]) < STICK_DEADBAND; 

                // Reset controller when moving into deadband
                if (inBand && !_inBandPrev) {
                    _velPid.reset();
                    didReset = true;
                }
                _inBandPrev = inBand;

                // Target velocity is a setpoint inside deadband, scaled constant outside
                float targetVelocity = inBand ?
                                       _posPid.compute(__yTarget, y) :
                                       PILOT_VELXY_MAX * demands[DEMANDS_ROLL];

                // Run velocity PID controller to get correction
                // XXX adjust for heading PSI
                demands[DEMANDS_ROLL] = _velPid.compute(targetVelocity, state->y[State::DY]);

                // If we re-entered deadband, we reset the target altitude.
                if (didReset) {
                    __yTarget = y;
                }
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            PositionHoldPid(const float Kp_pos=1, const float Kp_vel=2.0, const float Ki_vel=1.0, const float Kd_vel=0.5) 
            {
                _posPid.begin(Kp_pos, 0, 0);
                _velPid.begin(Kp_vel, Ki_vel, Kd_vel);

                _inBandPrev = false;
                __yTarget = 0;
            }

    };  // class PositionHoldPid

} // namespace hf
