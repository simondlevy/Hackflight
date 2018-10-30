/*
   poshold.hpp : PID-based position hold

   Copyright (c) 2018 Simon D. Levy

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
#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {


    class PositionHold : public PID_Controller {

        friend class Hackflight;

        private:

        // PID constants set by constructor
        float _posP;
        float _posrP;
        float _posrI;
        float _posrD;

        // Values modified in-flight
        float _positionSetpointX;
        float _positionSetpointY;
        bool  _inBandPrev;

        bool inBand(float demand)
        {
            return fabs(demand) < Receiver::STICK_DEADBAND; 
        }

        void resetErrors(void)
        {
            //_lastError = 0;
            //_deltaError = 0;
            //_integralError = 0;
        }

        // https://raw.githubusercontent.com/wiki/iNavFlight/inav/images/nav_poshold_pids_diagram.jpg
        float angleCorrection(float positionSetpoint, float actualPosition, float actualVelocity)
        {
            float positionError = positionSetpoint - actualPosition;
            float velocitySetpoint = _posP * positionError;
            float velocityError = actualVelocity - velocitySetpoint;
            float accelerationSetpoint = velocityError * _posrP;

            return accelerationSetpoint;
         }

        protected:

        virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) 
        {
            (void)currentTime;

            // Reset position setpoint if moved into stick deadband
            bool inBandCurr = inBand(demands.pitch) && inBand(demands.roll);
            if (inBandCurr && !_inBandPrev) {
                _positionSetpointX = state.positionX;
                _positionSetpointY = state.positionY;
                resetErrors();
            }
            _inBandPrev = inBandCurr;

            if (inBandCurr) {
                demands.pitch -= angleCorrection(_positionSetpointX, state.positionX, state.velocityForward);
                demands.roll  -= angleCorrection(_positionSetpointY, state.positionY, state.velocityRightward);
            }

            return inBandCurr;
        }

        virtual bool shouldFlashLed(void) override 
        {
            return true;
        }

        public:

        PositionHold(float posP, float posrP, float posrI, float posrD=0.0f)
        {
            _posP = posP;
            _posrP = posrP;
            _posrI = posrI;
            _posrD = posrD;

            resetErrors();

            _positionSetpointX = 0;
            _positionSetpointY = 0;

            _inBandPrev = false;
        }

    };  // class PositionHold

} // namespace hf
