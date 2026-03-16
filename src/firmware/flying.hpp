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

        private:

            static constexpr float FREQ_HZ = 25;

            static const uint32_t HYSTERESIS_THRESHOLD_MSEC = 2000;

            static constexpr float MOTOR_IDLE_MAX = 0.05;

        public:

            bool isFlying;

            FlyingCheck() = default;

            FlyingCheck(const bool isFlying, const uint32_t msec_prev)
                : isFlying(isFlying), _msec_prev(msec_prev) {}

            FlyingCheck& operator=(const FlyingCheck& other) = default;

             // We say we are flying if one or more motors are running over the idle
            // thrust.
            auto run(
                    const FlyingCheck & flyingCheck,
                    const uint32_t msec_curr,
                    const float * motorvals,
                    const uint8_t motor_count) -> FlyingCheck
            {
                if (msec_curr - flyingCheck._msec_prev > 1000/FREQ_HZ) {

                    auto isThrustOverIdle = false;

                    for (int i = 0; i < motor_count; ++i) {
                        if (motorvals[i] > MOTOR_IDLE_MAX) {
                            isThrustOverIdle = true;
                            break;
                        }
                    }

                    const auto msec_prev = isThrustOverIdle ? msec_curr :
                        flyingCheck._msec_prev;

                    const auto isFlying = (msec_prev > 0 &&
                            (msec_curr - msec_prev) < HYSTERESIS_THRESHOLD_MSEC);

                    return FlyingCheck(isFlying, msec_prev);
                }
                else {
                    return flyingCheck;
                }

            }

        private:

            uint32_t _msec_prev;
    };
}
