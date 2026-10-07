/*
   LED indicator support

   Copyright (C) 2026 Simon D. Levy

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

#pragma once

#include <stdint.h>

namespace hf {

    class Led {

        public:

            typedef enum {

                kStatusUnchanged, 
                kStatusOff, 
                kStatusOn, 

            } Status;

            Led(
                    const Status status,
                    const bool is_pulsing,
                    const uint32_t pulse_start_msec) 
                :
                    status_(status),
                    is_pulsing_(is_pulsing),
                    pulse_start_msec_(pulse_start_msec) {}

            Led() = default;

            Led(const Led & other) = default;

        private:

            Status status_;
            bool is_pulsing_;
            uint32_t pulse_start_msec_;
    };
}


