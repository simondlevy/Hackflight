/*
   Altitude hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

#include "pidcontroller.hpp"

#include <rft_closedloops/dofpid.hpp>

namespace hf {

    class AltitudeHoldPid : public PidController {

        private: 

            // Arbitrary constants: for details see http://ardupilot.org/copter/docs/altholdmode.html
            static constexpr float PILOT_VELZ_MAX = 2.5;
            static constexpr float STICK_DEADBAND = 0.20;   

            // P controller for position.  This will serve as the set-point for velocity PID.
            rft::DofPid _posPid;

            // PID controller for velocity
            rft::DofPid _velPid;

            // Will be reset each time we re-enter deadband.
            float _altitudeTarget = 0;

            // Tracks whether we just entered deadband
            bool _inBandPrev = false;

        protected:

            void modifyDemands(State * state, float * demands) override
            {
                bool didReset = false;
                float altitude = state->x[State::Z];

                // Is stick demand in deadband?
                bool inBand = fabs(demands[DEMANDS_THROTTLE]) < STICK_DEADBAND; 

                // Reset controller when moving into deadband
                if (inBand && !_inBandPrev) {
                    _velPid.reset();
                    didReset = true;
                }
                _inBandPrev = inBand;

                // Target velocity is a setpoint inside deadband, scaled constant outside
                float targetVelocity = inBand ?
                                       _posPid.compute(_altitudeTarget, altitude) :
                                       PILOT_VELZ_MAX * demands[DEMANDS_THROTTLE];

                // Run velocity PID controller to get correction
                demands[DEMANDS_THROTTLE] = _velPid.compute(targetVelocity, state->x[State::DZ]);

                // If we re-entered deadband, we reset the target altitude.
                if (didReset) {
                    _altitudeTarget = altitude;
                }
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            AltitudeHoldPid(const float Kp_pos=1, const float Kp_vel=0.75, const float Ki_vel=1.5, const float Kd_vel=0) 
            {
                _posPid.begin(Kp_pos, 0, 0);
                _velPid.begin(Kp_vel, Ki_vel, Kd_vel);

                _inBandPrev = false;
                _altitudeTarget = 0;
            }

    };  // class AltitudeHoldPid

} // namespace hf
