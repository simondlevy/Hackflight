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
            return airborne ? runpid(dt, psi, yaw) : 0;
        }

    private:

        static constexpr float KP = 6;
        static constexpr float KI = 1;
        static constexpr float KD = 0.35;
        static constexpr float ILIMIT = 360;
        static constexpr float DEMAND_MAX = 200;

        float _integral;
        float _previous;

        static float runpid(const float dt, const float psi, const float yaw) 
        {
            static float _target;
            static float _integral;
            static float _previous;

            _target = 1000 * cap(_target + DEMAND_MAX * yaw * dt);

            const auto error = cap(_target - psi*1000);

            _integral = Num::fconstrain(1000*(_integral + error * dt), ILIMIT * 1000); 

            auto deriv = dt > 0 ? 1000 * (error - _previous) / dt : 0;

            _previous = error;

            return (KP * error + KI * _integral + KD * deriv) / 1000;
        }

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
