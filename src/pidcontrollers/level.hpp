/*
   level.hpp : PID controller for Level mode

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

#include "debug.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "filters.hpp"

namespace hf {

    class Level : public PID_Controller {

        friend class Hackflight;

        private:
          
            const float FEED_FORWARD = 0.5;
            
            float PTerms[2];
            
            float _demandsToAngle;

        public:

            Level(float rollLevelP, float pitchLevelP, float maxAngle = 10)
            {
                PTerms[0] = rollLevelP;
                PTerms[1] = pitchLevelP;
                // roll and pitch demands go between [-0.5, 0.5] so, for a
                // given max angle, the following relation must hold true:
                // 0.5 * _demandsToAngle = maxAngle
                // Since we work in radians:
                // _demandsToAngle = deg2rad(maxAngle) * 2
                _demandsToAngle = 2* Filter::deg2rad(maxAngle);
            }

            Level(float rollPitchLevelP) : Level(rollPitchLevelP, rollPitchLevelP)
            {
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                float _demands[2] = {demands.roll, demands.pitch};
                for (int axis=0; axis<2; ++axis)
                {
                  float error = _demands[axis] * _demandsToAngle - state.eulerAngles[axis];
                  _demands[axis] = error * PTerms[axis] + FEED_FORWARD * _demands[axis];
                }
                
                demands.roll = _demands[0];
                demands.pitch = _demands[1];

                // We've always gotta do this!
                return true;
            }

    };  // class Level

} // namespace
