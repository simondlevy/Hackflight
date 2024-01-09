/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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
            RATE_25_HZ   = 25,
            RATE_30_HZ   = 30,
            RATE_33_HZ   = 33,
            RATE_50_HZ   = 50,
            RATE_100_HZ  = 100,
            RATE_250_HZ  = 250,
            RATE_500_HZ  = 500,
            RATE_1000_HZ = 1000,
        } rate_t ;

        static const rate_t RATE_MAIN_LOOP = RATE_1000_HZ;

        static bool rateDoExecute(const rate_t rate, const uint32_t tick)
        {
            return (tick % (RATE_MAIN_LOOP / rate)) == 0;
        }
};
