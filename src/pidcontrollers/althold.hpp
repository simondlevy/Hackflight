/*
   Altitude hold PID controller

   Copyright (c) 2021 Juan Gallostra and Simon D. Levy

   MIT License
*/

#pragma once

#include <RFT_state.hpp>
#include <RFT_closedloops/pidcontroller.hpp>

#include "demands.hpp"
#include "state.hpp"

namespace hf {

    class AltitudeHoldPid : public rft::PidController {

        private: 

            // Arbitrary constants: for details see http://ardupilot.org/copter/docs/altholdmode.html
            static constexpr float PILOT_VELZ_MAX  = 2.5f;
            static constexpr float STICK_DEADBAND = 0.10;   

            bool _inBandPrev = false;

            // P controller for position.  This will serve as the set-point for velocity PID.
            rft::DofPid _posPid;

            // PID controller for velocity
            rft::DofPid _velPid;

            // This will be reset each time we re-enter throttle deadband.
            float _altitudeTarget = 0;

        protected:

            void modifyDemands(rft::State * state, float * demands)
            {
                float * x = ((State *)state)->x;

                bool didReset = false;
                float altitude = x[State::STATE_Z];

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
                demands[DEMANDS_THROTTLE] = _velPid.compute(targetVelocity, x[State::STATE_DZ]);

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

            AltitudeHoldPid(const float Kp_pos, const float Kp_vel, const float Ki_vel, const float Kd_vel) 
            {
                _posPid.begin(Kp_pos, 0, 0);
                _velPid.begin(Kp_vel, Ki_vel, Kd_vel);

                _inBandPrev = false;
                _altitudeTarget = 0;
            }

    };  // class AltitudeHoldPid

} // namespace hf
