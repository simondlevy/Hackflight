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

class Timer {

    public:

        bool ready(const float freq)
        {
            const uint32_t msec_curr = millis();

            if (msec_curr - _msec_prev > 1000 / freq) {

                _msec_prev = msec_curr;

                return true;
            }

            return false;
        }
    
    private:

        uint32_t _msec_prev;
};
