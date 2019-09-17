/*
   PID controller for Level mode

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   Author: Juan Gallostra

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

#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "filters.hpp"
#include "anglebased.hpp"

namespace hf {

    class LevelPid : public PidController {

        friend class Hackflight;

        private:

            static constexpr float MAX_ANGLE_DEGREES = 10;
          
            AngleBased _rollPid;
            AngleBased _pitchPid;

        public:

            LevelPid(const float rollP, float const pitchP, 
                    float maxRollDegrees = MAX_ANGLE_DEGREES, float maxPitchDegrees = MAX_ANGLE_DEGREES)
            {
                _rollPid.init(rollP, maxRollDegrees);
                _pitchPid.init(pitchP, maxPitchDegrees);
            }

            LevelPid(const float Kp, const float maxAngleDegrees = MAX_ANGLE_DEGREES) 
                : LevelPid(Kp, Kp, maxAngleDegrees, maxAngleDegrees)
            {
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                demands.roll = _rollPid.compute(demands.roll, state.rotation[0]);
                demands.pitch = _pitchPid.compute(demands.pitch, state.rotation[1]);

                return true;
            }

    };  // class LevelPid

} // namespace
