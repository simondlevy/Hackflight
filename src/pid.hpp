/*
  PID control support for Hackflight
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
*/

#pragma once

#include <hackflight.hpp>
#include <utils.hpp>

class PID {

    public:

        float run_p(
                const float kp, 
                const float dt,
                const float target,
                const float actual)
        {
            return run(kp, 0, 0, 0, dt, target, actual, false);
        }

        float run_pi(
                const float kp, 
                const float ki, 
                const float ilimit, 
                const float dt,
                const float target,
                const float actual,
                const bool reset=false)
        {
            return run(kp, ki, 0, ilimit, dt, target, actual, reset);
        }

        float run_pd(
                const float kp, 
                const float kd, 
                const float dt,
                const float target,
                const float actual,
                const bool reset=false)
        {
            return run(kp, 0, kd, 0, dt, target, actual, reset);
        }

    private:

        float _integ;
        float _error;

        float run(
                const float kp, 
                const float ki, 
                const float kd, 
                const float ilimit,
                const float dt,
                const float target,
                const float actual,
                const bool reset)
        {
            const auto error = target - actual;

            const auto deriv = (error - _error) / dt;

            _error = reset ? 0 : error;

            _integ = reset ? 0 : 
                Utils::fconstrain(_integ + error * dt, -ilimit, +ilimit);

            return kp * error + ki * _integ + kd * deriv;
        }

};
