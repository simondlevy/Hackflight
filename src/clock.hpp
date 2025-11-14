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

#include <stdint.h>

class Clock {

    public:

        // Permitted frequencies
        typedef enum {
            FREQ_25_HZ   = 25,
            FREQ_50_HZ   = 50,
            FREQ_100_HZ  = 100,
            FREQ_500_HZ  = 500,
            FREQ_1000_HZ = 1000
        } rate_t ;

        static const rate_t IMU_FREQ = FREQ_1000_HZ;

        static bool rateDoExecute(const rate_t rate, const uint32_t tick)
        {
            return (tick % (IMU_FREQ / rate)) == 0;
        }
};
