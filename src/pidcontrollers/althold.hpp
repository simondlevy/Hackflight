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
            static constexpr float HOVER_THROTTLE  = 0.05f;
            static constexpr float VEL_WINDUP_MAX  = 0.40f;

            // P controller for position
            Pid _posPid;

            // PID controller for velocity
            Pid _velPid;

            // Minimum altitude, set by constructor
            float _minAltitude = 0;

            // PID constants set by constructor
            float _Kp_vel = 0;
            float _Ki_vel = 0;
            float _Kd_vel = 0;

            // Values modified in-flight
            float _posTarget = 0;
            bool  _inBandPrev = false;
            float _previousTime = 0;

            float _lastError = 0;
            float _integralError = 0;

            bool inBand(float demand)
            {
                return fabs(demand) < Receiver::STICK_DEADBAND; 
            }

            void resetErrors(void)
            {
                _lastError = 0;
                _integralError = 0;
            }

            bool gotCorrection(float demand, float posActual, float velActual, float currentTime, float & correction)
            {
                // Don't do anything until we have a positive deltaT
                float deltaT = currentTime - _previousTime;

                _previousTime = currentTime;
                if (deltaT == currentTime) return false;

                // Reset target if moved into stick deadband
                bool inBandCurr = inBand(demand);
                if (inBandCurr && !_inBandPrev) {
                    _posTarget = posActual;
                    resetErrors();
                }
                _inBandPrev = inBandCurr;

                if (!inBandCurr) return false;

                // Velocity target is output of position P controller
                float velTarget = _posPid.compute(_posTarget, posActual);

                float velError = velTarget - velActual;

                // Update error integral and error derivative
                _integralError = Filter::constrainAbs(_integralError + velError * deltaT, VEL_WINDUP_MAX);
                float deltaError = (velError - _lastError) / deltaT;
                _lastError = velError;

                // Compute control correction
                correction = _Kp_vel * velError + _Kd_vel * deltaError + _Ki_vel * _integralError;                       

                return true;
            }


        protected:

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                // Don't do anything till we've reached sufficient altitude
                if (state.location[2] < _minAltitude) return false;

                float correction = 0;
                if (gotCorrection(demands.throttle, state.location[2], state.inertialVel[2], currentTime, correction)) {
                    demands.throttle = correction + HOVER_THROTTLE;
                    return true;
                }

                return false;

            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            AltitudeHoldPid(const float Kp_pos, const float Kp_vel, const float Ki_vel, const float Kd_vel, const float minAltitude=0.1) 
                : _minAltitude(minAltitude)
            {
                _posPid.init(Kp_pos, 0, 0);
                _velPid.init(Kp_vel, Ki_vel, Kd_vel, 1, VEL_WINDUP_MAX);

                _Kp_vel = Kp_vel; 
                _Ki_vel = Ki_vel; 
                _Kd_vel = Kd_vel; 

                resetErrors();

                _posTarget = 0;
                _previousTime = 0;
                _inBandPrev = false;
            }

    };  // class AltitudeHoldPid

} // namespace hf
