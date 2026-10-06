/* Code for a debounced pushbutton switch
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>

namespace hf {

    class Pushbutton {

        private:

            static constexpr uint16_t kThreshold = 5;
            static constexpr uint32_t kDebounceDelayMsec = 50;

        public:

            Pushbutton(
                    const bool reading,
                    const uint32_t last_debounce_msec,
                    const uint8_t state) 
                : reading_(reading),
                last_debounce_msec_(last_debounce_msec),
                state_(state) {}

            Pushbutton() = default;

            Pushbutton(const Pushbutton & other) = default;

            auto Debounce(
                    const uint16_t analog_value, const uint32_t msec) -> bool
            {
                const auto reading = analog_value < kThreshold;

                if (reading != reading_) {
                    last_debounce_msec_ = msec;
                }

                if ((msec - last_debounce_msec_) > kDebounceDelayMsec) {

                    if (reading != state_) {
                        state_ = reading;
                    }
                }

                reading_ = reading;

                return state_;
            }

        private:

            bool reading_;
            uint32_t last_debounce_msec_;
            uint8_t state_;
    };
}


