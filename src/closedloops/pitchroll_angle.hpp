/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <pid.hpp>
#include <closedloop.hpp>
#include <num.hpp>

class PitchRollAngleController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate,
                const float kp=6,
                const float ki=3,
                const float kd=0)
        {
            ClosedLoopController::init(updateRate);

            initAxis(kp, ki, kd, _rollPid);

            initAxis(kp, ki, kd, _pitchPid);
        }

        /**
          * Demand is input as angles in degrees and output as angular
          * velocities in degrees per second:
          *
          * roll: right-down positive
          *
          * pitch: nose-up positive
          */
         virtual void run(const vehicleState_t & state, 
                demands_t & demands) override
        {
            demands.roll = _rollPid.run(demands.roll, state.phi);

            demands.pitch = _pitchPid.run(demands.pitch, state.theta);
        }

        void resetPids(void)
        {
            _rollPid.reset();

            _pitchPid.reset();
        }

    private:

        static constexpr float INTEGRAL_LIMIT = 20;
        static constexpr float FILTER_CUTOFF = 30;

        Pid _rollPid;
        Pid _pitchPid;

        void initAxis(
                const float kp, const float ki, const  float kd, Pid & pid)
        {
            pid.init(kp,  ki,  kd, 0, _dt, _updateRate, FILTER_CUTOFF, false);

            pid.setIntegralLimit(INTEGRAL_LIMIT);
        }
};
