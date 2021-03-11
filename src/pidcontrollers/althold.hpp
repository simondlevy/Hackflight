/*
   Altitude hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include "filters.hpp"
#include "pidcontroller.hpp"
#include "state.hpp"
#include "demands.hpp"

namespace hf {

    class AltitudeHoldPid : public PidController {

        private: 

            // Arbitrary constants: for details see http://ardupilot.org/copter/docs/altholdmode.html
            static constexpr float PILOT_VELZ_MAX  = 2.5f;
            static constexpr float STICK_DEADBAND = 0.10;   

            bool _inBandPrev = false;

            // P controller for position.  This will serve as the set-point for velocity PID.
            Pid _posPid;

            // PID controller for velocity
            Pid _velPid;

            // This will be reset each time we re-enter throttle deadband.
            float _altitudeTarget = 0;

        protected:

            void modifyDemands(state_t * state, demands_t & demands)
            {
                bool didReset = false;
                float altitude = state->x[STATE_Z];

                // Is stick demand in deadband?
                bool inBand = fabs(demands.throttle) < STICK_DEADBAND; 

                // Reset controller when moving into deadband
                if (inBand && !_inBandPrev) {
                    _velPid.reset();
                    didReset = true;
                }
                _inBandPrev = inBand;

                // Target velocity is a setpoint inside deadband, scaled constant outside
                float targetVelocity = inBand ? _posPid.compute(_altitudeTarget, altitude) : PILOT_VELZ_MAX * demands.throttle;

                // Run velocity PID controller to get correction
                demands.throttle = _velPid.compute(targetVelocity, state->x[STATE_DZ]);

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
