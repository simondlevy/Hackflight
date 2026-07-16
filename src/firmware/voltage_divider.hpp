/*
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

#include <Arduino.h>

#include <hackflight.h>

namespace hf {

    class VoltageDivider {

        private:

            uint8_t input_pin_;
            float r1_ohms_;
            float r2_ohms_;

        public:

            VoltageDivider(
                    const uint8_t input_pin,
                    const float r1_ohms,
                    const float r2_ohms)
                : input_pin_(input_pin),
                r1_ohms_(r1_ohms),
                r2_ohms_(r2_ohms) {}

            auto read() -> float
            {
                return analogRead(input_pin_) / 1024.f * 3.3 *
                    (r1_ohms_ + r2_ohms_) / r2_ohms_;
            }

    }; // class VoltageDivider

} // namespace hf
