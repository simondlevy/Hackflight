/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <Arduino.h>

namespace hf {

    class Timer {

        public:

            Timer( const uint32_t msec_prev,
                  const bool ready)
                : msec_prev_(msec_prev), ready_(ready) {}

            Timer() = default;

            Timer(const Timer &other) = default;

            static auto Update(
                    const Timer & timer,
                    const float freq,
                    const uint32_t msec_curr) -> Timer
            {
                const auto ready = msec_curr - timer.msec_prev_ > 1000 / freq;

                const auto msec_prev = ready ?  msec_curr : timer.msec_prev_;

                return Timer(msec_prev, ready);

            }

            static auto IsReady(const Timer & timer) -> bool
            {
                return timer.ready_;
            }

        private:

            uint32_t msec_prev_;
            bool ready_;
    };

}
