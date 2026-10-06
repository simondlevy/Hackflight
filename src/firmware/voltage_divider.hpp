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

        private:

            float r1_ohms_;
            float r2_ohms_;
            uint8_t nbits_;
            float signal_volts_;

        public:

            VoltageDivider(
                    const float r1_ohms,
                    const float r2_ohms,
                    const uint8_t nbits=10,
                    const float signal_volts=3.3)
                : r1_ohms_(r1_ohms),
                r2_ohms_(r2_ohms),
                nbits_(nbits),
                signal_volts_(signal_volts) {}

            auto Convert(const uint16_t rawval) -> float
            {
                return (float)rawval / (1 << nbits_) * signal_volts_ *
                    (r1_ohms_ + r2_ohms_) / r2_ohms_;
            }

    }; // class VoltageDivider

} // namespace hf
