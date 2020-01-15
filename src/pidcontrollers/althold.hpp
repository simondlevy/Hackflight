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
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class AltitudeHoldPid : public PidController {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            static constexpr float PILOT_VELZ_MAX  = 2.5f; // http://ardupilot.org/copter/docs/altholdmode.html

            // P controller for position.  This will serve as the set-point for velocity PID.
            Pid _posPid;

            // PID controller for velocity
            VelocityPid _velPid;

            // This will be reset each time we re-enter throttle deadband.
            float _altitudeTarget = 0;

        protected:

            void modifyDemands(state_t * state, demands_t * demands)
            {
                float altitude = state->location[2];

                // Run the velocity-based PID controller, using position-based PID controller output inside deadband, throttle-stick
                // proportion outside.  
                demands->throttle = _velPid.compute(demands->throttle, _posPid.compute(_altitudeTarget, altitude), PILOT_VELZ_MAX, state->inertialVel[2]);

                // If we re-entered deadband, we reset the target altitude.
                if (_velPid.didReset()) {
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
                _posPid.init(Kp_pos, 0, 0);
                _velPid.init(Kp_vel, Ki_vel, Kd_vel);

                _altitudeTarget = 0;
            }

    };  // class AltitudeHoldPid

} // namespace hf
