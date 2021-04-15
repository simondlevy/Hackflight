/*
   PID controller for Level mode

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

#include "state.hpp"
#include "demands.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class LevelPid : public PidController {

        private:

            // Helper class
            class _AnglePid : public Pid {

                private:

                    static constexpr float MAX_ANGLE_DEGREES = 45;

                    // Maximum roll pitch demand is +/-0.5, so to convert demand to 
                    // angle for error computation, we multiply by the folling amount:
                    float _demandMultiplier = 2 * Filter::deg2rad(MAX_ANGLE_DEGREES);

                public:

                    void begin(const float Kp) 
                    {
                        Pid::begin(Kp, 0, 0);
                    }

                    float compute(float demand, float angle)
                    {
                        return Pid::compute(demand*_demandMultiplier, angle);
                    }

            }; // class _AnglePid

            _AnglePid _rollPid;
            _AnglePid _pitchPid;

        public:

            LevelPid(float rollLevelP, float pitchLevelP)
            {
                _rollPid.begin(rollLevelP);
                _pitchPid.begin(pitchLevelP);
            }

            LevelPid(float rollPitchLevelP)
                : LevelPid(rollPitchLevelP, rollPitchLevelP)
            {
            }

            void modifyDemands(state_t * state, float * demands)
            {
                demands[DEMANDS_ROLL]  = _rollPid.compute(demands[DEMANDS_ROLL], state->x[STATE_PHI]);
                demands[DEMANDS_PITCH] = _pitchPid.compute(demands[DEMANDS_PITCH], state->x[STATE_THETA]);
            }

    };  // class LevelPid

} // namespace
