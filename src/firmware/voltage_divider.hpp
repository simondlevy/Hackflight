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

#include <hackflight.h>

namespace hf {

    class VoltageDivider {

        public:

            static auto Convert(
                    const float r1_ohms,
                    const float r2_ohms,
                    const uint16_t rawval,
                    const uint8_t nbits=10,
                    const float signal_volts=3.3) -> float
            {
                return (float)rawval / (1 << nbits) * signal_volts *
                    (r1_ohms + r2_ohms) / r2_ohms;
            }

    }; // class VoltageDivider

} // namespace hf
