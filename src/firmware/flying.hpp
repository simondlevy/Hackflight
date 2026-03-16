/*
   Copyright (C) 2011-2022 Bitcraze AB, 2026 Simon D. Levy

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

#include <hackflight.h>

namespace hf {

    class FlyingCheck {

        public:

            static const uint32_t FREQ_HZ = 25;

        private:

            static const uint32_t HYSTERESIS_THRESHOLD = 2000;

            static constexpr float MOTOR_IDLE_MAX = 0.01;

        public:

            // We say we are flying if one or more motors are running over the idle
            // thrust.
            static bool run(
                    const uint32_t msec_curr,
                    const float * motorvals,
                    const uint8_t motor_count)
            {
                auto isThrustOverIdle = false;

                for (int i = 0; i < motor_count; ++i) {
                    if (motorvals[i] > MOTOR_IDLE_MAX) {
                        isThrustOverIdle = true;
                        break;
                    }
                }

                static uint32_t _msec_prev;

                if (isThrustOverIdle) {
                    _msec_prev = msec_curr;
                }

                bool result = false;
                if (_msec_prev > 0) {
                    if ((msec_curr - _msec_prev) < HYSTERESIS_THRESHOLD) {
                        result = true;
                    }
                }

                return result;
            }
    };
}
