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
        
        private:

            static constexpr uint32_t kPulseDurationMsec = 50;

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

            static auto Update(
                    const Led & led,
                    const uint32_t msec,
                    const bool is_ready) -> Led
            {
                auto status = led.status_;
                auto is_pulsing = led.is_pulsing_;
                auto pulse_start_msec = led.pulse_start_msec_;

                if (is_ready) {
                    status = Led::kStatusOn;
                    is_pulsing = true;
                    pulse_start_msec = msec;
                }

                else if (led.is_pulsing_) {
                    if (msec - led.pulse_start_msec_ > kPulseDurationMsec) {
                        status = Led::kStatusOff;
                        is_pulsing = false;
                    }
                }

                else {
                    status = Led::kStatusUnchanged;
                }
 
                return Led(status, is_pulsing, pulse_start_msec);
            }

        //private:

            Status status_;
            bool is_pulsing_;
            uint32_t pulse_start_msec_;
    };
}


