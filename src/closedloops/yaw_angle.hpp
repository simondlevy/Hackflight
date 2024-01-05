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

class YawAngleController : public ClosedLoopController {

    public:

        void init(
                const Clock::rate_t updateRate, 
                const float kp=6,
                const float ki=1,
                const float kd=0.35)
        {
            ClosedLoopController::init(updateRate);

            _pid.init(kp,  ki,  kd, 0, _dt, _updateRate, CUTOFF_FREQ, false);

            _pid.setIntegralLimit(INTEGRAL_LIMIT);
        }

        /**
          * Demand is input as desired angle normalized to [-1,+1] and output
          * as degrees per second, both nose-right positive.
          */
         virtual void run(const vehicleState_t & state, 
                demands_t & demands) override 
        {
            static float _angleTarget;

            // Yaw angle psi is positive nose-left, whereas yaw demand is
            // positive nose-right.  Hence we negate the yaw demand to
            // accumulate the angle target.
            _angleTarget = cap(
                    _angleTarget - DEMAND_ANGLE_MAX * demands.yaw * _dt);

            const auto angleError = cap(_angleTarget - state.psi);

            _pid.setError(angleError);

            // Return the result negated, so demand will still be nose-right
            // positive
            demands.yaw = -_pid.run();

            if (demands.thrust == 0) {

                // Reset the calculated YAW angle for rate control
                _angleTarget = state.psi;
            }
        }

        void resetPids(void)
        {
            _pid.reset();
        }

    private:

        static constexpr float DEMAND_ANGLE_MAX = 200;
        static constexpr float CUTOFF_FREQ = 30;
        static constexpr float INTEGRAL_LIMIT = 360;

        static float cap(float angle) 
        {
            float result = angle;

            while (result > 180.0f) {
                result -= 360.0f;
            }

            while (result < -180.0f) {
                result += 360.0f;
            }

            return result;
        }

        Pid _pid;
};
