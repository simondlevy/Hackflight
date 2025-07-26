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

               static constexpr float DEMAND_MAX = 200;

                static float _target;

                _target = cap(_target + DEMAND_MAX * yaw * dt);

                // correction = oldpid(dt, _target, psi);
                correction = newpid(_target, psi);
            }

            return correction;
        }

    private:

        static float oldpid(
                const float dt, const float target, const float actual)
        {
            static constexpr float KP = 6;
            static constexpr float KI = 1;
            static constexpr float KD = 0.35;
            static constexpr float ILIMIT = 360;

            static float _integral;
            static float _previous;

            const auto error = target - actual;

            _integral = Num::fconstrain(_integral + error * dt, ILIMIT);

            auto deriv = dt > 0 ? (error - _previous) / dt : 0;

            _previous = error;

            return KP * error + KI * _integral + KD * deriv;
        }

        static float newpid(const float target, const float actual)
        {
            static constexpr float KP = 6;
            static constexpr float KI = 1;
            static constexpr float KD = 0.35;
            static constexpr float ILIMIT = 360;

            static float _integral;
            static float _previous;

            // encode
            const float target_encoded = target * 1000;
            const float actual_encoded = actual * 1000;

            const float error = target_encoded - actual_encoded;

            _integral = Num::fconstrain(_integral + error, ILIMIT);

            const float deriv = error - _previous;

            _previous = error;

            const float correction = (KP * error + KI * _integral + KD * deriv);

            csvdump(target_encoded, actual_encoded, error, _integral, deriv, correction);

            // decode
            return correction / 1000;
        }

        static void csvdump(const float target, const float actual,
                const float error, const float integral, const float deriv,
                const float correction)
        {
            static bool _ready;

            if (!_ready) {

                printf("target,actual,error,integral,derivative,correction\n");
                _ready = true;
            }

            printf("%f,%f,%f,%f,%f,%f\n", target, actual, error, integral, deriv, correction);
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
