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

            Timer(const float freq) : freq_(freq) { }

            auto Ready() -> bool
            {
                const uint32_t msec_curr = millis();

                if (msec_curr - msec_prev_ > 1000 / freq_) {

                    msec_prev_ = msec_curr;

                    return true;
                }

                return false;
            }

        private:

            float freq_;

            uint32_t msec_prev_;
    };

}
