/*
   PID controller support for pitch, roll in acro mode

   Supports yaw stabilization and acro mode

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

namespace hf {

    class AnglePid {

        friend class Hackflight;

        private: 


        float computePid(float PTerm, float ITerm, float DTerm, float rate)
        {
            PTerm = (PTerm * _demandsScale - rate) * _P;

            return PTerm + ITerm + DTerm;
        }

        protected:

        float _demandsScale;

        void resetIntegral(void)
        {
            _errorI = 0;
        }

        public:

        AnglePid(float P, float I, float demandsScale=1.0f) 
            : _P(P), _I(I), _demandsScale(demandsScale)
        {
            // Zero-out previous values for D term
            _lastError   = 0;

            // Convert degree parameters to radians for use later
            _bigAngularVel = Filter::deg2rad(BIG_DEGREES_PER_SECOND);

            // Initialize gyro error integral
            resetIntegral();
        }

    };  // class AnglePid

} // namespace hf
