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

            static auto ready(const uint32_t msec_curr, const uint32_t msec_prev,
                    const float freq) -> bool
            {
                return msec_curr - msec_prev > 1000 / freq;
            }

            auto ready(const float freq) -> bool
            {
                const uint32_t msec_curr = millis();

                if (ready(msec_curr, _msec_prev, freq)) {

                    _msec_prev = msec_curr;

                    return true;
                }

                return false;
            }

            static auto getDt() -> float
            {
                const auto usec_curr = micros();      
                static uint32_t _usec_prev;
                const float dt = (usec_curr - _usec_prev)/1000000.0;
                _usec_prev = usec_curr;

                return dt;
            }

            static void runDelayLoop(const uint32_t usec_curr, 
                    const uint32_t loop_freq_hz=2000)
            {
                float invFreq = 1.0 / loop_freq_hz * 1000000.0;
                uint32_t checker = micros();

                while (invFreq > (checker - usec_curr)) {
                    checker = micros();
                }
            }

        private:

            uint32_t _msec_prev;
    };

}
