/*
   althold.hpp : Altitude hold PID controller

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

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>
#include <cmath>

#include "receiver.hpp"
#include "filters.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class AltitudeHold : public PID_Controller {

        friend class Hackflight;

        private: 

              // Arbitrary constants
              const float WINDUP_MAX             = 0.40f;
              const float HOVER_THROTTLE         = 0.05f;
              const float MIN_ALTITUDE           = 0.10f;

              // PID constants set in constructor
              float _altHoldP;
              float _altHoldVelP;
              float _altHoldVelI;
              float _altHoldVelD;
            
              // Required constants
              float _lastError;
              float _deltaError;
              float _integralError;
              float _velocityTarget;
              float _altitudeTarget;
              uint32_t _previousTime;
            

              void resetErrors(void)
              {
                  _lastError = 0;
                  _deltaError = 0;
                  _integralError = 0;
              }


          protected:
            
              bool modifyDemands(state_t & state, demands_t & demands)
              {
                uint32_t _currentTime = micros();
                float dt = (_currentTime - _previousTime) / 1000000.0f;
                _previousTime = _currentTime;
                _altitudeTarget = 0.20f;
                // Compute vertical velocity setpoint and error
                _velocityTarget = (_altitudeTarget - state.altitude) * _altHoldP;
                float velocityError = _velocityTarget - state.variometer;
                // Update error integral and error derivative
                _integralError = Filter::constrainAbs(_integralError + velocityError * dt, WINDUP_MAX);
                _deltaError = (velocityError - _lastError) / dt;
                _lastError = velocityError;
                float throttleCorrection = _altHoldVelP * velocityError +
                                           _altHoldVelD * _deltaError +
                                           _altHoldVelI * _integralError;

                demands.throttle = HOVER_THROTTLE + throttleCorrection;

                return true;
              }
              
              virtual bool shouldFlashLed(void) override 
              {
                  return true;
              }


        public:

            AltitudeHold(float altHoldP, float altHoldVelP, float altHoldVelI, float altHoldVelD) :
                _altHoldP(altHoldP), 
                _altHoldVelP(altHoldVelP), 
                _altHoldVelI(altHoldVelI),
                _altHoldVelD(altHoldVelD) 
            {
              // Initialize errors
              resetErrors();
              _previousTime = micros();
            }

    };  // class AltitudeHold

} // namespace
