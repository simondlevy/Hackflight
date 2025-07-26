/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <num.hpp>

class YawAngleController {

    public:

        /**
         * Input is desired angle (deg) estimated actual angle (deg) from EKF;
         ouputput is angles-per-second demand sent to YawRateController.
         */
        static float run(
                const bool airborne,  // ignore this
                const float dt,       // can be a constant if needed
                const float psi,      // estimated angle
                const float yaw)      // desired angle
        {
            float correction = 0;

            if (airborne) {

                static constexpr float KP = 6;
                static constexpr float KI = 1;
                static constexpr float KD = 0.35;
                static constexpr float ILIMIT = 360;
                static constexpr float DEMAND_MAX = 200;

                static float _target;

                _target = cap(_target + DEMAND_MAX * yaw * dt);

                correction = newpid(KP, KI, KD, ILIMIT, dt, _target, psi);
            }

            return correction;
        }

    private:

        static float oldpid(
                const float kp,
                const float ki,
                const float kd,
                const float ilimit,
                const float dt,
                const float target,
                const float actual)
        {
            static float _integral;
            static float _previous;

            const auto error = target - actual;

            _integral = Num::fconstrain(_integral + error * dt, ilimit);

            auto deriv = dt > 0 ? (error - _previous) / dt : 0;

            _previous = error;

            return kp * error + ki * _integral + kd * deriv;
        }

        static float newpid(
                const float kp,
                const float ki,
                const float kd,
                const float ilimit,
                const float dt,
                const float target,
                const float actual)
        {
            static float _integral;
            static float _previous;

            const auto error = 1000 * target - 1000 * actual;

            _integral = Num::fconstrain(_integral + error * dt, ilimit);

            auto deriv = dt > 0 ? (error - _previous) / dt : 0;

            _previous = error;

            // return kp * error + ki * _integral + kd * deriv;
            return (kp * error) / 1000;
        }

        // Keep angle in (0, 360)
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
};
