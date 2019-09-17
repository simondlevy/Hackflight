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
#include "pid.hpp"
#include "filters.hpp"

namespace hf {

    // Helper class
    class _AnglePid : public Pid {

        private:

            static constexpr float FEED_FORWARD = 0.5;

            float _Kp = 0;

            float _demandToAngle = 0;

        public:

            void init(const float Kp, const float maxAngle) 
            {
                _Kp = Kp;

                // Roll and pitch demands go between [-0.5, 0.5] so, for a  given max angle, the following relation must hold true:
                // 0.5 * _demandToAngle = maxAngle
                _demandToAngle = 2* Filter::deg2rad(maxAngle);
            }

            float compute(float demand, float angle)
            {

                float error = demand * _demandToAngle - angle;

                return error * _Kp + FEED_FORWARD * demand;
            }

    }; // class _AnglePid

    class LevelPid : public PidController {

        private:

            _AnglePid _rollPid;
            _AnglePid _pitchPid;

        public:

            LevelPid(float rollLevelP, float pitchLevelP, float maxAngle = 10)
            {
                _rollPid.init(rollLevelP, maxAngle);
                _pitchPid.init(pitchLevelP, maxAngle);
            }

            LevelPid(float rollPitchLevelP)
                : LevelPid(rollPitchLevelP, rollPitchLevelP)
            {
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                demands.roll  = _rollPid.compute(demands.roll, state.rotation[0]); 
                demands.pitch = _pitchPid.compute(demands.pitch, state.rotation[1]);

                return true;
            }

    };  // class LevelPid

} // namespace
