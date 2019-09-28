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

#include "receiver.hpp"
#include "filters.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "pid.hpp"

namespace hf {

    class AltitudeHoldPid : public PidController {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            static constexpr float VEL_WINDUP_MAX  = 0.40f;
            static constexpr float MIN_ALTITUDE    = 5.0f;

            // P controller for position
            Pid _posPid;

            // PID controller for velocity
            Pid _velPid;

            // Values modified in-flight
            float _altitudeTarget = 0;
            bool  _inBandPrev = false;
            float _previousTime = 0;

            bool _inBand = false;

        protected:

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                float altitude = state.location[2];

                if (altitude < MIN_ALTITUDE) return false;

                _inBand = fabs(demands.throttle) < STICK_DEADBAND; 

                // Reset controller when moving into deadband
                if (_inBand && !_inBandPrev) {
                    _altitudeTarget = altitude;
                    _velPid.reset();
                }

                _inBandPrev = _inBand;

                //debugline("alt: %f  tgt: %f", altitude, _altitudeTarget);

                if (_inBand) {

                    // Velocity target is output of position P controller
                    float velTarget = _posPid.compute(_altitudeTarget, altitude);

                    // Run velocity PID controller to get correction
                    demands.throttle = _velPid.compute(velTarget, state.inertialVel[2], currentTime);

                    return true;
                }

                return false;
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            AltitudeHoldPid(const float Kp_pos, const float Kp_vel, const float Ki_vel, const float Kd_vel) 
            {
                _inBand = false;

                _posPid.init(Kp_pos, 0, 0);
                _velPid.init(Kp_vel, Ki_vel, Kd_vel, 1, VEL_WINDUP_MAX);

                _altitudeTarget = 0;
                _previousTime = 0;
                _inBand = false;
                _inBandPrev = false;
            }

    };  // class AltitudeHoldPid

} // namespace hf
